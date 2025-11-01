/**
 * @file pipeline.ts
 * @description Core execution pipeline for SynapseRobo SDK
 * Manages Action → Verify → Execute → Log flow with Promise queues,
 * timeout handling, emergency stops, and telemetry hooks.
 */

import { Subject, Observable, from, of, throwError, timer, interval, queueScheduler } from 'rxjs';
import { map, catchError, timeout, retry, switchMap, takeUntil, filter, tap, mergeMap, observeOn, finalize } from 'rxjs/operators';
import { v4 as uuidv4 } from 'uuid';
import { logger } from '@utils/logger';
import { ActionPlan, AIAction, PipelineStep, PipelineResult, PipelineConfig, TelemetryData, EmergencyStop } from '@utils/types';
import { RobotController } from './robotController';
import { SolanaClient } from './solanaClient';
import { AIEngine } from './aiEngine';

interface PipelineQueueItem {
  action: AIAction;
  planId: string;
  attempt: number;
  timestamp: number;
}

interface ExecutionContext {
  robotController: RobotController;
  solanaClient: SolanaClient;
  aiEngine: AIEngine;
  telemetry: TelemetryData[];
}

export class ExecutionPipeline {
  private config: PipelineConfig;
  private queue: PipelineQueueItem[] = [];
  private processing = false;
  private emergencyStop = false;
  private stopSignal = new Subject<void>();
  private telemetryStream = new Subject<TelemetryData>();
  private stepSubjects = {
    verify: new Subject<PipelineStep>(),
    execute: new Subject<PipelineStep>(),
    log: new Subject<PipelineStep>(),
  };

  /**
   * Creates a new execution pipeline
   * @param config - Pipeline configuration
   */
  constructor(config: Partial<PipelineConfig> = {}) {
    this.config = {
      maxConcurrent: config.maxConcurrent ?? 1,
      actionTimeout: config.actionTimeout ?? 30000,
      verificationTimeout: config.verificationTimeout ?? 10000,
      retryAttempts: config.retryAttempts ?? 3,
      emergencyStopEnabled: config.emergencyStopEnabled ?? true,
      telemetryEnabled: config.telemetryEnabled ?? true,
      blockchainVerification: config.blockchainVerification ?? true,
    };

    this.startProcessingLoop();
  }

  /**
   * Enqueues an action plan for execution
   * @param plan - Action plan to execute
   * @param context - Execution context with required components
   * @returns Observable of pipeline results
   */
  public enqueuePlan(
    plan: ActionPlan,
    context: ExecutionContext
  ): Observable<PipelineResult> {
    const results$ = new Subject<PipelineResult>();

    plan.actions.forEach((action, index) => {
      const queueItem: PipelineQueueItem = {
        action,
        planId: plan.id,
        attempt: 0,
        timestamp: Date.now() + index * 100, // Stagger execution
      };
      this.queue.push(queueItem);
    });

    // Sort queue by timestamp
    this.queue.sort((a, b) => a.timestamp - b.timestamp);

    logger.info('Action plan enqueued', {
      planId: plan.id,
      actionCount: plan.actions.length,
      queueLength: this.queue.length,
    });

    // Subscribe to completion
    let completed = 0;
    const total = plan.actions.length;

    const subscription = this.stepSubjects.execute.subscribe(step => {
      if (step.planId === plan.id && step.status === 'completed') {
        completed++;
        if (completed === total) {
          results$.next({
            planId: plan.id,
            success: true,
            completedActions: completed,
            timestamp: Date.now(),
          });
          results$.complete();
          subscription.unsubscribe();
        }
      }
    });

    return results$.asObservable();
  }

  /**
   * Executes a single action through the pipeline
   * @param action - Action to execute
   * @param context - Execution context
   * @returns Pipeline result
   */
  public async executeAction(
    action: AIAction,
    context: ExecutionContext
  ): Promise<PipelineResult> {
    const executionId = uuidv4();
    const startTime = Date.now();

    logger.info('Starting pipeline execution', {
      executionId,
      actionId: action.id,
      type: action.type,
    });

    try {
      // Step 1: Verify
      const verification = await this.verifyAction(action, context, executionId);
      if (!verification.success) {
        return this.handlePipelineFailure(action, verification.error!, executionId, startTime);
      }

      // Step 2: Execute
      const execution = await this.executeRobotAction(action, context, executionId);
      if (!execution.success) {
        return this.handlePipelineFailure(action, execution.error!, executionId, startTime);
      }

      // Step 3: Log to blockchain
      if (this.config.blockchainVerification) {
        const blockchain = await this.logToBlockchain(action, execution.result!, context, executionId);
        if (!blockchain.success) {
          logger.warn('Blockchain logging failed but action succeeded', {
            executionId,
            error: blockchain.error,
          });
        }
      }

      const result: PipelineResult = {
        executionId,
        actionId: action.id,
        success: true,
        timestamp: Date.now(),
        duration: Date.now() - startTime,
        telemetry: this.collectTelemetry(context),
      };

      this.emitTelemetry(result);
      logger.info('Pipeline execution completed successfully', {
        executionId,
        duration: result.duration,
      });

      return result;
    } catch (error) {
      return this.handlePipelineFailure(action, error as Error, executionId, startTime);
    }
  }

  /**
   * Triggers emergency stop
   */
  public emergencyStop(): void {
    if (!this.config.emergencyStopEnabled) return;

    this.emergencyStop = true;
    this.stopSignal.next();
    
    logger.error('EMERGENCY STOP TRIGGERED - Halting all robot operations');

    // Clear queue
    this.queue = [];
    
    // Notify all components
    this.stepSubjects.execute.next({
      status: 'emergency_stop',
      timestamp: Date.now(),
    });
  }

  /**
   * Resumes execution after emergency stop
   */
  public resume(): void {
    if (!this.emergencyStop) return;

    this.emergencyStop = false;
    logger.info('Pipeline execution resumed after emergency stop');
  }

  /**
   * Gets current pipeline status
   */
  public getStatus(): {
    queueLength: number;
    processing: boolean;
    emergencyStop: boolean;
    config: PipelineConfig;
  } {
    return {
      queueLength: this.queue.length,
      processing: this.processing,
      emergencyStop: this.emergencyStop,
      config: this.config,
    };
  }

  /**
   * Subscribes to telemetry stream
   */
  public getTelemetryStream(): Observable<TelemetryData> {
    return this.telemetryStream.asObservable();
  }

  private startProcessingLoop(): void {
    interval(100).pipe(
      observeOn(queueScheduler),
      filter(() => this.queue.length > 0 && !this.processing && !this.emergencyStop),
      tap(() => this.processQueue())
    ).subscribe();
  }

  private async processQueue(): Promise<void> {
    if (this.processing || this.emergencyStop) return;

    this.processing = true;
    const item = this.queue.shift();

    if (item && item.attempt < this.config.retryAttempts!) {
      try {
        // This would integrate with full context in real implementation
        await this.delay(50);
        
        this.stepSubjects.execute.next({
          actionId: item.action.id,
          planId: item.planId,
          status: 'completed',
          timestamp: Date.now(),
        });
      } catch (error) {
        item.attempt++;
        if (item.attempt < this.config.retryAttempts!) {
          this.queue.unshift(item); // Retry
        }
      }
    }

    this.processing = false;
  }

  private async verifyAction(
    action: AIAction,
    context: ExecutionContext,
    executionId: string
  ): Promise<{ success: boolean; error?: Error }> {
    const step: PipelineStep = {
      executionId,
      actionId: action.id,
      step: 'verify',
      status: 'processing',
      timestamp: Date.now(),
    };

    this.stepSubjects.verify.next(step);

    try {
      // AI safety verification
      const aiValidation = context.aiEngine.validateAction(action);
      if (!aiValidation.valid && aiValidation.violations.length > 0) {
        throw new Error(`Safety violation: ${aiValidation.violations[0].message}`);
      }

      // Robot capability check
      const canExecute = await context.robotController.canExecuteAction(action);
      if (!canExecute) {
        throw new Error('Robot cannot execute action with current state');
      }

      // Pre-condition verification
      if (action.preconditions) {
        for (const condition of action.preconditions) {
          const satisfied = await this.checkPrecondition(condition, context);
          if (!satisfied) {
            throw new Error(`Precondition not met: ${condition}`);
          }
        }
      }

      step.status = 'completed';
      this.stepSubjects.verify.next(step);

      return { success: true };
    } catch (error) {
      step.status = 'failed';
      step.error = error as Error;
      this.stepSubjects.verify.next(step);

      return { success: false, error: error as Error };
    }
  }

  private async executeRobotAction(
    action: AIAction,
    context: ExecutionContext,
    executionId: string
  ): Promise<{ success: boolean; result?: any; error?: Error }> {
    const step: PipelineStep = {
      executionId,
      actionId: action.id,
      step: 'execute',
      status: 'processing',
      timestamp: Date.now(),
    };

    this.stepSubjects.execute.next(step);

    try {
      const result = await context.robotController.executeAction(action).pipe(
        timeout(this.config.actionTimeout!),
        takeUntil(this.stopSignal),
        catchError(error => {
          logger.error('Action execution timeout or error', { executionId, error });
          return throwError(() => error);
        })
      ).toPromise();

      step.status = 'completed';
      step.result = result;
      this.stepSubjects.execute.next(step);

      return { success: true, result };
    } catch (error) {
      step.status = 'failed';
      step.error = error as Error;
      this.stepSubjects.execute.next(step);

      return { success: false, error: error as Error };
    }
  }

  private async logToBlockchain(
    action: AIAction,
    result: any,
    context: ExecutionContext,
    executionId: string
  ): Promise<{ success: boolean; error?: Error }> {
    const step: PipelineStep = {
      executionId,
      actionId: action.id,
      step: 'blockchain_log',
      status: 'processing',
      timestamp: Date.now(),
    };

    this.stepSubjects.log.next(step);

    try {
      const logData = {
        actionId: action.id,
        type: action.type,
        parameters: action.parameters,
        result: JSON.stringify(result),
        timestamp: Date.now(),
        robotId: context.robotController.getActiveRobotId(),
      };

      await context.solanaClient.logAction(logData);

      step.status = 'completed';
      this.stepSubjects.log.next(step);

      return { success: true };
    } catch (error) {
      step.status = 'failed';
      step.error = error as Error;
      this.stepSubjects.log.next(step);

      return { success: false, error: error as Error };
    }
  }

  private handlePipelineFailure(
    action: AIAction,
    error: Error,
    executionId: string,
    startTime: number
  ): PipelineResult {
    const result: PipelineResult = {
      executionId,
      actionId: action.id,
      success: false,
      error: error.message,
      timestamp: Date.now(),
      duration: Date.now() - startTime,
    };

    this.emitTelemetry(result);
    logger.error('Pipeline execution failed', {
      executionId,
      actionId: action.id,
      error: error.message,
    });

    return result;
  }

  private async checkPrecondition(
    condition: string,
    context: ExecutionContext
  ): Promise<boolean> {
    // Implement condition checking logic
    // This would interface with robot sensors and state
    return true;
  }

  private collectTelemetry(context: ExecutionContext): TelemetryData[] {
    if (!this.config.telemetryEnabled) return [];

    const robotState = context.robotController.getRobotState();
    const network = context.solanaClient.getNetworkStatus();

    return [
      {
        type: 'robot_state',
        data: robotState,
        timestamp: Date.now(),
      },
      {
        type: 'network_status',
        data: network,
        timestamp: Date.now(),
      },
    ];
  }

  private emitTelemetry(result: PipelineResult): void {
    if (!this.config.telemetryEnabled) return;

    this.telemetryStream.next({
      type: 'pipeline_result',
      data: result,
      timestamp: Date.now(),
    });
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}