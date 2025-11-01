/**
 * @file taskManager.ts
 * @description Core task manager for SynapseRobo SDK
 * Handles job scheduling, priority queues, retry policies, 
 * robot idle handling, AI override rules, and task lifecycle.
 */

import { Subject, Observable, from, of, throwError, timer, interval, BehaviorSubject, queueScheduler } from 'rxjs';
import { map, catchError, timeout, retry, switchMap, takeUntil, filter, tap, mergeMap, observeOn, finalize, take } from 'rxjs/operators';
import { v4 as uuidv4 } from 'uuid';
import { logger } from '@utils/logger';
import { 
  Task, 
  TaskStatus, 
  TaskPriority, 
  TaskResult, 
  TaskConfig, 
  ScheduleRule,
  RetryPolicy,
  AIOverrideRule
} from '@utils/types';
import { AIEngine } from './aiEngine';
import { RobotController } from './robotController';
import { ExecutionPipeline } from './pipeline';

interface TaskQueueItem {
  task: Task;
  scheduledTime: number;
  attempts: number;
  id: string;
}

interface ActiveTask {
  task: Task;
  startTime: number;
  observable?: Observable<any>;
  subscription?: any;
}

export class TaskManager {
  private config: TaskConfig;
  private taskQueue: TaskQueueItem[] = [];
  private activeTasks = new Map<string, ActiveTask>();
  private completedTasks = new Map<string, TaskResult>();
  private scheduleRules: ScheduleRule[] = [];
  private retryPolicies: Map<string, RetryPolicy> = new Map();
  private aiOverrideRules: AIOverrideRule[] = [];
  private processing = false;
  private taskSubject = new Subject<TaskResult>();
  private statusSubject = new BehaviorSubject<Map<string, TaskStatus>>(new Map());
  private aiEngine?: AIEngine;
  private robotController?: RobotController;
  private pipeline?: ExecutionPipeline;

  /**
   * Creates a new task manager
   * @param config - Task manager configuration
   */
  constructor(config: Partial<TaskConfig> = {}) {
    this.config = {
      maxConcurrentTasks: config.maxConcurrentTasks ?? 3,
      defaultPriority: config.defaultPriority ?? TaskPriority.MEDIUM,
      idleTimeout: config.idleTimeout ?? 300000, // 5 minutes
      cleanupInterval: config.cleanupInterval ?? 3600000, // 1 hour
      aiOverrideEnabled: config.aiOverrideEnabled ?? true,
      schedulingEnabled: config.schedulingEnabled ?? true,
    };

    this.initializeDefaultRetryPolicies();
    this.startProcessingLoop();
    this.startIdleHandler();
    this.startCleanupJob();
  }

  /**
   * Sets AI engine for override rules
   * @param aiEngine - AI engine instance
   */
  public setAIEngine(aiEngine: AIEngine): void {
    this.aiEngine = aiEngine;
    logger.info('AI Engine connected to Task Manager');
  }

  /**
   * Sets robot controller for task execution
   * @param controller - Robot controller instance
   */
  public setRobotController(controller: RobotController): void {
    this.robotController = controller;
    logger.info('Robot Controller connected to Task Manager');
  }

  /**
   * Sets execution pipeline
   * @param pipeline - Execution pipeline instance
   */
  public setPipeline(pipeline: ExecutionPipeline): void {
    this.pipeline = pipeline;
    logger.info('Execution Pipeline connected to Task Manager');
  }

  /**
   * Submits a new task
   * @param task - Task to execute
   * @returns Task ID
   */
  public submitTask(task: Omit<Task, 'id' | 'status' | 'createdAt'>): string {
    const taskId = uuidv4();
    const now = Date.now();

    const fullTask: Task = {
      ...task,
      id: taskId,
      status: TaskStatus.PENDING,
      createdAt: now,
      priority: task.priority || this.config.defaultPriority!,
    };

    const queueItem: TaskQueueItem = {
      task: fullTask,
      scheduledTime: now,
      attempts: 0,
      id: taskId,
    };

    this.taskQueue.push(queueItem);
    this.sortQueue();
    this.updateStatus();

    logger.info('Task submitted', { 
      taskId, 
      type: task.type, 
      priority: fullTask.priority,
      queueLength: this.taskQueue.length 
    });

    return taskId;
  }

  /**
   * Schedules a recurring task
   * @param rule - Schedule rule
   * @param task - Task template
   */
  public scheduleTask(rule: ScheduleRule, task: Omit<Task, 'id' | 'status' | 'createdAt'>): void {
    this.scheduleRules.push({ ...rule, task });
    logger.info('Task scheduled', { 
      ruleId: rule.id, 
      cron: rule.cron, 
      taskType: task.type 
    });
  }

  /**
   * Cancels a task
   * @param taskId - Task ID
   */
  public cancelTask(taskId: string): boolean {
    const queueIndex = this.taskQueue.findIndex(item => item.id === taskId);
    if (queueIndex > -1) {
      this.taskQueue.splice(queueIndex, 1);
      this.updateStatus();
      logger.info('Task cancelled from queue', { taskId });
      return true;
    }

    const active = this.activeTasks.get(taskId);
    if (active && active.subscription) {
      active.subscription.unsubscribe();
      active.task.status = TaskStatus.CANCELLED;
      this.activeTasks.delete(taskId);
      this.updateStatus();
      logger.info('Active task cancelled', { taskId });
      return true;
    }

    return false;
  }

  /**
   * Gets task status
   * @param taskId - Task ID
   */
  public getTaskStatus(taskId: string): TaskStatus | undefined {
    // Check active tasks
    const active = this.activeTasks.get(taskId);
    if (active) return active.task.status;

    // Check queue
    const queued = this.taskQueue.find(item => item.id === taskId);
    if (queued) return queued.task.status;

    // Check completed
    const completed = this.completedTasks.get(taskId);
    if (completed) return completed.success ? TaskStatus.COMPLETED : TaskStatus.FAILED;

    return undefined;
  }

  /**
   * Gets all task statuses
   */
  public getAllStatuses(): Map<string, TaskStatus> {
    const statuses = new Map<string, TaskStatus>();

    // Active tasks
    for (const [id, active] of this.activeTasks) {
      statuses.set(id, active.task.status);
    }

    // Queued tasks
    for (const item of this.taskQueue) {
      statuses.set(item.id, item.task.status);
    }

    // Completed tasks (last 100)
    for (const [id, result] of Array.from(this.completedTasks.entries()).slice(-100)) {
      statuses.set(id, result.success ? TaskStatus.COMPLETED : TaskStatus.FAILED);
    }

    return statuses;
  }

  /**
   * Subscribes to task results
   */
  public getTaskStream(): Observable<TaskResult> {
    return this.taskSubject.asObservable();
  }

  /**
   * Subscribes to status updates
   */
  public getStatusStream(): Observable<Map<string, TaskStatus>> {
    return this.statusSubject.asObservable();
  }

  /**
   * Handles robot idle state
   */
  public handleRobotIdle(): void {
    if (!this.robotController) return;

    const activeRobotId = this.robotController.getActiveRobotId();
    if (!activeRobotId) return;

    const idleTask = this.createIdleTask(activeRobotId);
    this.submitTask(idleTask);

    logger.info('Robot idle task triggered', { robotId: activeRobotId });
  }

  private initializeDefaultRetryPolicies(): void {
    this.retryPolicies.set('default', {
      maxAttempts: 3,
      delayMs: 1000,
      backoffFactor: 2,
      retryableErrors: ['timeout', 'network_error', 'transient'],
    });

    this.retryPolicies.set('exploration', {
      maxAttempts: 5,
      delayMs: 2000,
      backoffFactor: 1.5,
      retryableErrors: ['obstacle_detected', 'path_blocked'],
    });

    this.retryPolicies.set('payment', {
      maxAttempts: 2,
      delayMs: 5000,
      backoffFactor: 1,
      retryableErrors: ['insufficient_funds', 'rate_limit'],
    });
  }

  private startProcessingLoop(): void {
    interval(100).pipe(
      observeOn(queueScheduler),
      filter(() => this.canProcessMoreTasks() && !this.processing),
      tap(() => this.processNextTask())
    ).subscribe();
  }

  private startIdleHandler(): void {
    if (!this.config.idleTimeout || !this.robotController) return;

    interval(10000).pipe(
      tap(() => {
        const activeRobotId = this.robotController!.getActiveRobotId();
        if (!activeRobotId) return;

        const lastActive = this.getLastTaskTime(activeRobotId);
        const now = Date.now();

        if (now - lastActive > this.config.idleTimeout!) {
          this.handleRobotIdle();
        }
      })
    ).subscribe();
  }

  private startCleanupJob(): void {
    interval(this.config.cleanupInterval!).pipe(
      tap(() => this.cleanupCompletedTasks())
    ).subscribe();
  }

  private canProcessMoreTasks(): boolean {
    return this.activeTasks.size < this.config.maxConcurrentTasks!;
  }

  private async processNextTask(): Promise<void> {
    if (this.processing || !this.canProcessMoreTasks()) return;

    this.processing = true;
    const item = this.taskQueue.shift();

    if (!item) {
      this.processing = false;
      return;
    }

    // Apply scheduling delay
    const now = Date.now();
    if (item.scheduledTime > now) {
      this.taskQueue.unshift(item);
      this.processing = false;
      return;
    }

    // Update task status
    item.task.status = TaskStatus.RUNNING;
    this.updateStatus();

    const activeTask: ActiveTask = {
      task: item.task,
      startTime: Date.now(),
    };

    this.activeTasks.set(item.id, activeTask);

    try {
      // Check AI override rules
      if (this.config.aiOverrideEnabled && this.aiEngine) {
        const override = await this.checkAIOverride(item.task);
        if (override) {
          logger.info('AI override applied', { taskId: item.id, override: override.type });
          item.task = override.newTask;
        }
      }

      // Execute task
      const result = await this.executeTask(item.task, item.attempts);
      
      // Handle success
      const taskResult: TaskResult = {
        taskId: item.id,
        success: true,
        result,
        timestamp: Date.now(),
        duration: Date.now() - activeTask.startTime,
      };

      this.completeTask(item.id, taskResult);

    } catch (error) {
      // Handle failure with retry
      const shouldRetry = this.shouldRetryTask(item, error as Error);
      
      if (shouldRetry) {
        item.attempts++;
        item.scheduledTime = Date.now() + this.calculateRetryDelay(item);
        this.taskQueue.push(item);
        this.sortQueue();
        
        logger.warn('Task will retry', { 
          taskId: item.id, 
          attempt: item.attempts,
          delay: item.scheduledTime - Date.now()
        });
      } else {
        const taskResult: TaskResult = {
          taskId: item.id,
          success: false,
          error: (error as Error).message,
          timestamp: Date.now(),
          duration: Date.now() - activeTask.startTime,
        };

        this.completeTask(item.id, taskResult);
      }
    } finally {
      this.activeTasks.delete(item.id);
      this.processing = false;
      this.updateStatus();
    }
  }

  private async executeTask(task: Task, attempts: number): Promise<any> {
    logger.info('Executing task', { 
      taskId: task.id, 
      type: task.type, 
      attempt: attempts + 1 
    });

    // Route task based on type
    switch (task.type) {
      case 'exploration':
      case 'retrieval':
      case 'interactive':
        return this.executeAIPlanTask(task);
      
      case 'maintenance':
        return this.executeMaintenanceTask(task);
      
      case 'payment':
        return this.executePaymentTask(task);
      
      case 'idle':
        return this.executeIdleTask(task);
      
      default:
        if (task.customHandler) {
          return task.customHandler(task.payload);
        }
        throw new Error(`Unknown task type: ${task.type}`);
    }
  }

  private async executeAIPlanTask(task: Task): Promise<any> {
    if (!this.aiEngine || !this.pipeline) {
      throw new Error('AI Engine or Pipeline not configured');
    }

    const prompt = task.payload?.prompt || `Execute ${task.type} task`;
    const plan = await this.aiEngine.generateActionPlan(prompt);

    const result$ = this.pipeline.enqueuePlan(plan, {
      robotController: this.robotController!,
      solanaClient: this.pipeline['solanaClient'], // Internal access
      aiEngine: this.aiEngine,
      telemetry: [],
    });

    return result$.pipe(
      timeout(300000), // 5 minute timeout
      take(1)
    ).toPromise();
  }

  private async executeMaintenanceTask(task: Task): Promise<any> {
    // Simulate maintenance
    await this.delay(5000);
    return { status: 'maintenance_completed', timestamp: Date.now() };
  }

  private async executePaymentTask(task: Task): Promise<any> {
    if (!this.pipeline) throw new Error('Pipeline required for payments');
    
    const solanaClient = this.pipeline['solanaClient'];
    if (!solanaClient) throw new Error('Solana client not available');

    const request = task.payload;
    return solanaClient.processPayment(request);
  }

  private async executeIdleTask(task: Task): Promise<any> {
    // Robot performs idle behavior
    if (this.robotController) {
      const commands = [
        { type: 'TURN_HEAD', parameters: { pan: 30, tilt: 0 } },
        { type: 'TURN_HEAD', parameters: { pan: -30, tilt: 0 } },
        { type: 'WAVE', parameters: { arm: 'right' } },
      ];

      for (const cmd of commands) {
        await this.robotController.executeCommand(cmd).pipe(take(1)).toPromise();
        await this.delay(2000);
      }
    }

    return { status: 'idle_behavior_completed' };
  }

  private async checkAIOverride(task: Task): Promise<{ type: string; newTask: Task } | null> {
    if (!this.aiEngine || !this.config.aiOverrideEnabled) return null;

    // Simple override check based on robot state
    const robotState = this.robotController?.getRobotState();
    if (robotState && robotState.battery < 15) {
      const overrideTask: Task = {
        ...task,
        id: uuidv4(),
        type: 'maintenance',
        priority: TaskPriority.HIGH,
        payload: { action: 'charge' },
        status: TaskStatus.PENDING,
        createdAt: Date.now(),
      };

      return { type: 'low_battery', newTask: overrideTask };
    }

    return null;
  }

  private shouldRetryTask(item: TaskQueueItem, error: Error): boolean {
    const policy = this.retryPolicies.get(item.task.type) || this.retryPolicies.get('default')!;
    const errorType = this.classifyError(error);

    return item.attempts < policy.maxAttempts && 
           policy.retryableErrors.includes(errorType);
  }

  private calculateRetryDelay(item: TaskQueueItem): number {
    const policy = this.retryPolicies.get(item.task.type) || this.retryPolicies.get('default')!;
    const baseDelay = policy.delayMs;
    const backoff = Math.pow(policy.backoffFactor, item.attempts);
    
    return baseDelay * backoff + Math.random() * 1000;
  }

  private classifyError(error: Error): string {
    const message = error.message.toLowerCase();
    
    if (message.includes('timeout')) return 'timeout';
    if (message.includes('network') || message.includes('connection')) return 'network_error';
    if (message.includes('rate') || message.includes('limit')) return 'rate_limit';
    if (message.includes('insufficient')) return 'insufficient_funds';
    
    return 'unknown';
  }

  private completeTask(taskId: string, result: TaskResult): void {
    this.completedTasks.set(taskId, result);
    
    if (this.completedTasks.size > 1000) {
      // Keep only recent tasks
      const entries = Array.from(this.completedTasks.entries());
      this.completedTasks = new Map(entries.slice(-500));
    }

    this.taskSubject.next(result);
    logger.info('Task completed', { 
      taskId, 
      success: result.success,
      duration: result.duration 
    });
  }

  private sortQueue(): void {
    this.taskQueue.sort((a, b) => {
      // Higher priority first
      if (a.task.priority !== b.task.priority) {
        return b.task.priority - a.task.priority;
      }
      
      // Earlier scheduled time first
      return a.scheduledTime - b.scheduledTime;
    });
  }

  private getLastTaskTime(robotId: string): number {
    let lastTime = 0;

    // Check active tasks
    for (const active of this.activeTasks.values()) {
      if (active.task.payload?.robotId === robotId) {
        lastTime = Math.max(lastTime, active.startTime);
      }
    }

    // Check recent completed tasks
    for (const result of this.completedTasks.values()) {
      if (result.result?.robotId === robotId) {
        lastTime = Math.max(lastTime, result.timestamp);
      }
    }

    return lastTime || Date.now();
  }

  private createIdleTask(robotId: string): Omit<Task, 'id' | 'status' | 'createdAt'> {
    return {
      type: 'idle',
      priority: TaskPriority.LOW,
      payload: { robotId },
    };
  }

  private cleanupCompletedTasks(): void {
    const cutoff = Date.now() - 24 * 60 * 60 * 1000; // 24 hours
    
    for (const [id, result] of this.completedTasks) {
      if (result.timestamp < cutoff) {
        this.completedTasks.delete(id);
      }
    }

    logger.debug('Completed tasks cleanup', { remaining: this.completedTasks.size });
  }

  private updateStatus(): void {
    this.statusSubject.next(this.getAllStatuses());
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}