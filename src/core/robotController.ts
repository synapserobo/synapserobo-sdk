/**
 * @file robotController.ts
 * @description Core robot controller for SynapseRobo SDK
 * Manages multiple robot instances, motion queues, kinematics, command interpretation,
 * real-time execution, and safety systems.
 */

import { Subject, Observable, from, of, throwError, interval, BehaviorSubject } from 'rxjs';
import { map, catchError, timeout, switchMap, takeUntil, filter, tap, mergeMap, finalize } from 'rxjs/operators';
import { v4 as uuidv4 } from 'uuid';
import { logger } from '@utils/logger';
import { Robot, RobotConfig, RobotState, MotionCommand, KinematicsResult, CommandResult, ControllerConfig } from '@utils/types';
import { Humanoid } from '@robots/humanoid';
import { ArmBot } from '@robots/armBot';
import { HardwareAdapter } from '@adapters/hardwareAdapter';
import { SimulationAdapter } from '@adapters/simulationAdapter';

interface MotionQueueItem {
  command: MotionCommand;
  priority: number;
  timestamp: number;
  id: string;
}

interface RobotInstance {
  robot: Robot;
  state: RobotState;
  motionQueue: MotionQueueItem[];
  isActive: boolean;
  lastHeartbeat: number;
}

export class RobotController {
  private config: ControllerConfig;
  private robots: Map<string, RobotInstance> = new Map();
  private activeRobotId: string | null = null;
  private motionProcessing = false;
  private emergencyStop = false;
  private stopSignal = new Subject<void>();
  private stateSubject = new BehaviorSubject<Map<string, RobotState>>(new Map());
  private commandSubject = new Subject<CommandResult>();

  /**
   * Creates a new robot controller
   * @param config - Controller configuration
   */
  constructor(config: Partial<ControllerConfig> = {}) {
    this.config = {
      simulation: config.simulation ?? false,
      timeout: config.timeout ?? 30000,
      maxVelocity: config.maxVelocity ?? 0.3,
      safetyEnabled: config.safetyEnabled ?? true,
      maxQueueSize: config.maxQueueSize ?? 100,
      heartbeatInterval: config.heartbeatInterval ?? 1000,
    };

    this.startHeartbeatMonitor();
    this.startMotionProcessor();
  }

  /**
   * Adds a robot to the controller
   * @param id - Unique robot identifier
   * @param robot - Robot instance
   * @returns Success indicator
   */
  public add  addRobot(id: string, robot: Robot): boolean {
    if (this.robots.has(id)) {
      logger.warn('Robot already exists', { id });
      return false;
    }

    const instance: RobotInstance = {
      robot,
      state: robot.getState(),
      motionQueue: [],
      isActive: false,
      lastHeartbeat: Date.now(),
    };

    this.robots.set(id, instance);
    
    if (!this.activeRobotId) {
      this.activeRobotId = id;
      instance.isActive = true;
    }

    logger.info('Robot added to controller', { id, type: robot.constructor.name });
    this.updateState();

    return true;
  }

  /**
   * Removes a robot from the controller
   * @param id - Robot identifier
   */
  public removeRobot(id: string): void {
    const instance = this.robots.get(id);
    if (!instance) return;

    // Stop all motion
    instance.motionQueue = [];
    
    this.robots.delete(id);
    
    if (this.activeRobotId === id) {
      this.activeRobotId = this.robots.keys().next().value || null;
      if (this.activeRobotId) {
        this.robots.get(this.activeRobotId)!.isActive = true;
      }
    }

    logger.info('Robot removed from controller', { id });
    this.updateState();
  }

  /**
   * Sets the active robot for command execution
   * @param id - Robot identifier
   */
  public setActiveRobot(id: string): boolean {
    if (!this.robots.has(id)) {
      logger.error('Cannot set active robot - not found', { id });
      return false;
    }

    if (this.activeRobotId) {
      this.robots.get(this.activeRobotId)!.isActive = false;
    }

    this.activeRobotId = id;
    this.robots.get(id)!.isActive = true;
    
    logger.info('Active robot changed', { id });
    this.updateState();
    
    return true;
  }

  /**
   * Executes a motion command on the active robot
   * @param command - Motion command
   * @returns Observable of command result
   */
  public executeCommand(command: MotionCommand): Observable<CommandResult> {
    const robotId = this.activeRobotId;
    if (!robotId) {
      return throwError(() => new Error('No active robot'));
    }

    const instance = this.robots.get(robotId)!;
    const commandId = uuidv4();

    // Validate command
    if (!this.validateCommand(command, instance.state)) {
      return throwError(() => new Error('Invalid command parameters'));
    }

    const queueItem: MotionQueueItem = {
      command,
      priority: command.priority || 0,
      timestamp: Date.now(),
      id: commandId,
    };

    instance.motionQueue.push(queueItem);
    this.sortMotionQueue(instance);

    logger.debug('Command queued', { 
      robotId, 
      commandId, 
      type: command.type,
      queueLength: instance.motionQueue.length 
    });

    // Return observable that resolves when command completes
    return this.commandSubject.pipe(
      filter(result => result.commandId === commandId),
      takeUntil(timer(this.config.timeout!)),
      tap(result => logger.info('Command completed', { commandId, success: result.success }))
    );
  }

  /**
   * Executes a JSON command string
   * @param jsonCommand - JSON command string
   * @returns Observable of result
   */
  public executeJSONCommand(jsonCommand: string): Observable<CommandResult> {
    try {
      const command = JSON.parse(jsonCommand) as MotionCommand;
      return this.executeCommand(command);
    } catch (error) {
      return throwError(() => new Error(`Invalid JSON command: ${error}`));
    }
  }

  /**
   * Checks if an action can be executed
   * @param action - AI action
   * @returns Promise of boolean
   */
  public async canExecuteAction(action: any): Promise<boolean> {
    if (!this.activeRobotId) return false;

    const instance = this.robots.get(this.activeRobotId)!;
    const state = instance.state;

    // Basic capability checks
    switch (action.type) {
      case 'WALK':
      case 'MOVE_FORWARD':
      case 'MOVE_BACKWARD':
        return state.capabilities.walking && state.battery > 20;
      
      case 'GRIP':
        return state.capabilities.gripping && state.gripper?.available;
      
      case 'LIFT':
        return state.capabilities.lifting && state.arms?.length === 2;
      
      case 'TURN_HEAD':
        return state.capabilities.headMovement;
      
      default:
        return true;
    }
  }

  /**
   * Executes an AI action by converting to motion commands
   * @param action - AI action
   * @returns Observable of result
   */
  public executeAction(action: any): Observable<any> {
    const commands = this.convertActionToCommands(action);
    
    return from(commands).pipe(
      mergeMap(command => this.executeCommand(command), this.config.maxConcurrent || 1),
      tap(result => {
        if (!result.success) {
          logger.warn('Action command failed', { actionId: action.id, error: result.error });
        }
      }),
      finalize(() => logger.info('AI action execution completed', { actionId: action.id }))
    );
  }

  /**
   * Triggers emergency stop on all robots
   */
  public emergencyStop(): void {
    this.emergencyStop = true;
    this.stopSignal.next();

    for (const [id, instance] of this.robots) {
      instance.motionQueue = [];
      instance.robot.emergencyStop();
      
      logger.error('Emergency stop executed', { robotId: id });
    }

    this.updateState();
  }

  /**
   * Resumes robot operations
   */
  public resume(): void {
    this.emergencyStop = false;
    logger.info('Robot operations resumed');
  }

  /**
   * Gets current state of all robots
   */
  public getAllStates(): Map<string, RobotState> {
    const states = new Map<string, RobotState>();
    
    for (const [id, instance] of this.robots) {
      states.set(id, instance.robot.getState());
    }
    
    return states;
  }

  /**
   * Gets state of active robot
   */
  public getRobotState(): RobotState {
    if (!this.activeRobotId) {
      throw new Error('No active robot');
    }
    
    return this.robots.get(this.activeRobotId)!.robot.getState();
  }

  /**
   * Gets active robot ID
   */
  public getActiveRobotId(): string | null {
    return this.activeRobotId;
  }

  /**
   * Subscribes to robot state updates
   */
  public getStateStream(): Observable<Map<string, RobotState>> {
    return this.stateSubject.asObservable();
  }

  /**
   * Subscribes to command results
   */
  public getCommandStream(): Observable<CommandResult> {
    return this.commandSubject.asObservable();
  }

  private startHeartbeatMonitor(): void {
    interval(this.config.heartbeatInterval!).pipe(
      tap(() => {
        const now = Date.now();
        for (const [id, instance] of this.robots) {
          if (now - instance.lastHeartbeat > this.config.heartbeatInterval! * 3) {
            logger.warn('Robot heartbeat timeout', { id });
            instance.isActive = false;
          } else {
            instance.lastHeartbeat = now;
          }
        }
        this.updateState();
      })
    ).subscribe();
  }

  private startMotionProcessor(): void {
    interval(50).pipe(
      filter(() => !this.motionProcessing && !this.emergencyStop),
      tap(() => this.processMotionQueues())
    ).subscribe();
  }

  private async processMotionQueues(): Promise<void> {
    if (this.motionProcessing || this.emergencyStop) return;

    this.motionProcessing = true;

    for (const [id, instance] of this.robots) {
      if (instance.motionQueue.length === 0 || !instance.isActive) continue;

      const item = instance.motionQueue[0];
      const robot = instance.robot;

      try {
        // Execute command based on type
        let result: any;
        
        switch (item.command.type) {
          case 'WALK':
            result = await robot.walk(
              item.command.parameters.distance,
              item.command.parameters.speed
            );
            break;
          
          case 'ROTATE':
            result = await robot.rotate(
              item.command.parameters.angle,
              item.command.parameters.speed
            );
            break;
          
          case 'LIFT':
            result = await robot.lift(
              item.command.parameters.height,
              item.command.parameters.arm
            );
            break;
          
          case 'GRIP':
            result = await robot.grip(
              item.command.parameters.force,
              item.command.parameters.duration
            );
            break;
          
          case 'TURN_HEAD':
            result = await robot.turnHead(
              item.command.parameters.pan,
              item.command.parameters.tilt
            );
            break;
          
          case 'STAND':
            result = await robot.stand();
            break;
          
          case 'SIT':
            result = await robot.sit();
            break;
          
          case 'WAVE':
            result = await robot.wave(item.command.parameters.arm);
            break;
          
          default:
            result = { success: false, error: 'Unknown command type' };
        }

        // Remove from queue
        instance.motionQueue.shift();

        // Emit result
        this.commandSubject.next({
          commandId: item.id,
          success: result.success !== false,
          result,
          timestamp: Date.now(),
          robotId: id,
        });

      } catch (error) {
        instance.motionQueue.shift();
        
        this.commandSubject.next({
          commandId: item.id,
          success: false,
          error: (error as Error).message,
          timestamp: Date.now(),
          robotId: id,
        });
      }
    }

    this.motionProcessing = false;
    this.updateState();
  }

  private sortMotionQueue(instance: RobotInstance): void {
    instance.motionQueue.sort((a, b) => {
      if (a.priority !== b.priority) {
        return b.priority - a.priority;
      }
      return a.timestamp - b.timestamp;
    });
  }

  private validateCommand(command: MotionCommand, state: RobotState): boolean {
    if (this.emergencyStop) return false;

    // Safety checks
    if (this.config.safetyEnabled) {
      if (command.parameters.speed && command.parameters.speed > this.config.maxVelocity!) {
        return false;
      }
      
      if (command.parameters.force && command.parameters.force > 20) {
        return false;
      }
      
      if (state.battery < 10) {
        logger.warn('Low battery - command rejected', { battery: state.battery });
        return false;
      }
    }

    return true;
  }

  private convertActionToCommands(action: any): MotionCommand[] {
    const commands: MotionCommand[] = [];
    const baseId = uuidv4();

    switch (action.type) {
      case 'MOVE_FORWARD':
      case 'WALK':
        commands.push({
          id: `${baseId}-walk`,
          type: 'WALK',
          parameters: {
            distance: action.parameters?.distance || 1.0,
            speed: Math.min(action.parameters?.speed || 0.2, this.config.maxVelocity!),
          },
          priority: action.priority || 0,
        });
        break;

      case 'GRIP':
        commands.push({
          id: `${baseId}-grip`,
          type: 'GRIP',
          parameters: {
            force: Math.min(action.parameters?.force || 15, 20),
            duration: action.parameters?.duration || 2,
          },
          priority: action.priority || 5,
        });
        break;

      case 'LIFT':
        commands.push({
          id: `${baseId}-lift`,
          type: 'LIFT',
          parameters: {
            height: action.parameters?.height || 0.3,
            arm: action.parameters?.arm || 'both',
          },
          priority: action.priority || 3,
        });
        break;

      default:
        commands.push({
          id: `${baseId}-unknown`,
          type: 'STAND',
          parameters: {},
          priority: action.priority || 0,
        });
    }

    return commands;
  }

  private updateState(): void {
    const states = new Map<string, RobotState>();
    
    for (const [id, instance] of this.robots) {
      states.set(id, instance.robot.getState());
    }
    
    this.stateSubject.next(states);
  }
}