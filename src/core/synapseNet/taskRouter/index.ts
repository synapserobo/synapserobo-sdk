import { Registry, TargetNode, RobotCapabilities, SimulationCapabilities } from './registry';
import { TaskQueue, QueueEntry, Task, TaskStatus } from './queue';
import { v4 as uuidv4 } from 'uuid';
import { EventEmitter } from 'events';

// Main route decision result interface
export interface RouteDecision {
  targetId: string;
  targetType: 'robot' | 'simulation';
  routeTimestamp: number;
  estimatedStartTime: number;
  taskId: string;
  priority: number;
  fallbackUsed: boolean;
}

// Task router configuration
export interface TaskRouterConfig {
  maxRecentTasks: number;          
  defaultPriority: number;     
  simulationWeight: number;     
  robotWeight: number;          
  taskTimeoutMs: number;         
  healthCheckIntervalMs: number;   
}

// Default configuration values
const DEFAULT_CONFIG: TaskRouterConfig = {
  maxRecentTasks: 1000,
  defaultPriority: 5,
  simulationWeight: 0.3,
  robotWeight: 0.7,
  taskTimeoutMs: 30000,
  healthCheckIntervalMs: 10000
};

// Task router events
export interface TaskRouterEvents {
  'taskAssigned': (decision: RouteDecision) => void;
  'taskRouted': (task: Task, target: TargetNode) => void;
  'targetUnavailable': (task: Task, reason: string) => void;
  'duplicateTaskDetected': (taskId: string, existingEntry: QueueEntry) => void;
}

// Main task router class
export class TaskRouter extends EventEmitter {
  private registry: Registry;
  private taskQueue: TaskQueue;
  private recentTasks: Map<string, number>; 
  private config: TaskRouterConfig;
  private isRunning: boolean;
  private processingInterval: NodeJS.Timeout | null;
  
  constructor(config: Partial<TaskRouterConfig> = {}) {
    super();
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.registry = new Registry();
    this.taskQueue = new TaskQueue();
    this.recentTasks = new Map();
    this.isRunning = false;
    this.processingInterval = null;
    this.startProcessingLoop();
  }
  
  // Start the internal processing loop
  private startProcessingLoop(): void {
    if (this.isRunning) return;
    this.isRunning = true;
    this.processingInterval = setInterval(() => {
      this.processQueue();
      this.cleanupRecentTasks();
    }, 1000); 
  }
  
  // Stop the processing loop
  public stop(): void {
    this.isRunning = false;
    if (this.processingInterval) {
      clearInterval(this.processingInterval);
      this.processingInterval = null;
    }
  }
  
  // Clean up old entries from recent tasks map
  private cleanupRecentTasks(): void {
    const now = Date.now();
    const timeout = this.config.taskTimeoutMs * 2; 
    for (const [taskId, timestamp] of this.recentTasks.entries()) {
      if (now - timestamp > timeout) {
        this.recentTasks.delete(taskId);
      }
    }
    // Enforce maximum size
    if (this.recentTasks.size > this.config.maxRecentTasks) {
      const entries = Array.from(this.recentTasks.entries());
      entries.sort((a, b) => a[1] - b[1]);
      const toRemove = entries.slice(0, entries.length - this.config.maxRecentTasks);
      toRemove.forEach(([taskId]) => this.recentTasks.delete(taskId));
    }
  }
  
  // Main task routing method
  public routeTask(task: Partial<Task>): RouteDecision | null {
    // Generate task ID if not provided
    const taskId = task.id || `task_${Date.now()}_${uuidv4().slice(0, 8)}`;
    const fullTask: Task = {
      id: taskId,
      type: task.type || 'unknown',
      payload: task.payload || {},
      priority: task.priority || this.config.defaultPriority,
      createdAt: task.createdAt || Date.now(),
      timeoutMs: task.timeoutMs || this.config.taskTimeoutMs,
      metadata: task.metadata || {},
      requiredCapabilities: task.requiredCapabilities || {}
    };
    
    // Check for replay protection
    if (this.recentTasks.has(taskId)) {
      const existingEntry = this.taskQueue.findTask(taskId);
      if (existingEntry) {
        this.emit('duplicateTaskDetected', taskId, existingEntry);
        return null; // Reject duplicate task
      }
    }
    
    // Select target based on deterministic rules
    const target = this.registry.selectAvailableTarget(
      fullTask.requiredCapabilities,
      fullTask.priority
    );
    
    if (!target) {
      this.emit('targetUnavailable', fullTask, 'No available targets matching requirements');
      // Queue task for later processing
      const queueEntry = this.taskQueue.enqueueTask(fullTask);
      return null;
    }
    
    // Create route decision
    const decision: RouteDecision = {
      targetId: target.id,
      targetType: target.type,
      routeTimestamp: Date.now(),
      estimatedStartTime: Date.now() + 100, // Small processing delay
      taskId: fullTask.id,
      priority: fullTask.priority,
      fallbackUsed: target.isFallback || false
    };
    
    // Add to recent tasks for replay protection
    this.recentTasks.set(taskId, Date.now());
    
    // Queue the task with assigned target
    const queueEntry = this.taskQueue.enqueueTask(fullTask, target.id);
    
    this.emit('taskRouted', fullTask, target);
    this.emit('taskAssigned', decision);
    
    return decision;
  }
  
  // Register a robot with capabilities
  public registerRobot(
    robotId: string,
    capabilities: RobotCapabilities,
    load: number = 0
  ): boolean {
    return this.registry.registerRobot(robotId, capabilities, load);
  }
  
  // Register a simulation node
  public registerSimulationNode(
    nodeId: string,
    capabilities: SimulationCapabilities,
    load: number = 0
  ): boolean {
    return this.registry.registerSimulationNode(nodeId, capabilities, load);
  }
  
  // Unregister a target
  public unregisterTarget(targetId: string): boolean {
    return this.registry.unregisterTarget(targetId);
  }
  
  // Update target status
  public updateTargetStatus(targetId: string, isOnline: boolean, load?: number): boolean {
    return this.registry.updateTargetStatus(targetId, isOnline, load);
  }
  
  // Process the task queue
  private processQueue(): void {
    const now = Date.now();
    const stalledTasks = this.taskQueue.getStalledTasks(now);
    stalledTasks.forEach(entry => {
      this.taskQueue.retryTask(entry.task.id);
      this.routeTask(entry.task);
    });
    
    const nextEntry = this.taskQueue.dequeueTask();
    if (nextEntry) {
      this.executeTask(nextEntry);
    }
  }
  
  private executeTask(entry: QueueEntry): void {
    setTimeout(() => {
      if (this.taskQueue.updateTaskStatus(entry.task.id, 'completed')) {
        this.recentTasks.delete(entry.task.id);
      }
    }, 100);
  }
  
  public onTaskAssigned(handler: (decision: RouteDecision) => void): void {
    this.on('taskAssigned', handler);
  }
  
  public getStats(): {
    queueLength: number;
    registeredTargets: number;
    onlineTargets: number;
    recentTasksCount: number;
  } {
    const registryStats = this.registry.getStats();
    return {
      queueLength: this.taskQueue.getQueueLength(),
      registeredTargets: registryStats.totalTargets,
      onlineTargets: registryStats.onlineTargets,
      recentTasksCount: this.recentTasks.size
    };
  }
  public clear(): void {
    this.taskQueue.clear();
    this.recentTasks.clear();
    this.registry.clear();
  }
}