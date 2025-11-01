/**
 * Example: AI-Powered Robot Automation with SynapseRobo SDK
 * 
 * This example demonstrates:
 * - AI-driven decision making for robots
 * - Real-time environment perception
 * - Adaptive behavior based on sensor data
 * - Multi-step task planning and execution
 * - Integration with blockchain for verification
 */

import { AIEngine } from '../src/core/aiEngine';
import { RobotController } from '../src/core/robotController';
import { Humanoid } from '../src/robots/humanoid';
import { SolanaClient } from '../src/core/solanaClient';
import { HardwareAdapter } from '../src/adapters/hardwareAdapter';
import { SimulationAdapter } from '../src/adapters/simulationAdapter';
import { logger } from '../src/utils/logger';
import { AI_MODELS, COMMAND_TYPES } from '../src/constants';
import { Keypair } from '@solana/web3.js';

class AIAutomation {
  private aiEngine: AIEngine;
  private robotController: RobotController;
  private solanaClient: SolanaClient;
  private humanoid: Humanoid;
  private isRunning: boolean = false;

  constructor() {
    this.initializeAISystem();
  }

  private initializeAISystem(): void {
    logger.info('Initializing AI Automation System...');

    // Initialize AI Engine
    this.aiEngine = new AIEngine({
      model: 'LOCAL_LLM',
      temperature: 0.7,
      maxTokens: 1000,
      timeout: 30000
    });

    // Initialize hardware and robot
    const simulationAdapter = new SimulationAdapter();
    const hardwareAdapter = new HardwareAdapter(simulationAdapter);

    this.humanoid = new Humanoid(hardwareAdapter, {
      height: 1.75,
      armLength: 0.7,
      legLength: 0.85,
      maxStepHeight: 0.15,
      balanceEnabled: true
    });

    this.robotController = new RobotController({
      simulation: true,
      timeout: 30000,
      maxVelocity: 0.3,
      safetyEnabled: true
    });

    // Initialize Solana client for blockchain verification
    this.solanaClient = new SolanaClient({
      network: 'devnet',
      commitment: 'confirmed',
      maxRetries: 3,
      retryDelay: 1000,
      wallet: Keypair.generate()
    });

    logger.info('AI Automation system initialized successfully');
  }

  public async start(): Promise<void> {
    try {
      this.isRunning = true;
      logger.info('Starting AI-powered automation...');

      // Connect all systems
      await this.connectSystems();
      
      // Run AI automation scenarios
      await this.runRoomExploration();
      await this.runObjectRetrieval();
      await this.runInteractiveTask();

      logger.info('AI automation completed successfully');

    } catch (error) {
      logger.error('AI automation failed', error as Error);
      await this.handleError(error as Error);
    } finally {
      await this.cleanup();
    }
  }

  private async connectSystems(): Promise<void> {
    logger.info('Connecting AI, robot, and blockchain systems...');

    await this.aiEngine.initialize();
    await this.humanoid.connect();
    await this.robotController.addRobot('ai_humanoid', this.humanoid);
    await this.solanaClient.connect();

    logger.info('All systems connected and ready');
  }

  private async runRoomExploration(): Promise<void> {
    logger.info('Starting AI-powered room exploration...');

    const explorationPrompt = `
      You are controlling a humanoid robot exploring an unknown room.
      Your task is to systematically explore the room to:
      1. Identify all objects in the room
      2. Map the room layout
      3. Identify potential obstacles and hazards
      4. Find the most interesting object to examine closer

      Current robot state: ${JSON.stringify(this.humanoid.getState())}
      Available actions: move_forward, move_backward, turn_left, turn_right, look_around, examine_object

      Provide a sequence of actions to explore the room effectively.
    `;

    const explorationPlan = await this.aiEngine.generateActionPlan(explorationPrompt);
    
    logger.info('AI generated exploration plan', {
      actionCount: explorationPlan.actions.length,
      estimatedDuration: explorationPlan.estimatedDuration
    });

    // Execute the AI-generated plan
    for (const action of explorationPlan.actions) {
      if (!this.isRunning) break;

      logger.info(`Executing AI action: ${action.description}`);
      
      const result = await this.executeAIAction(action);
      
      if (result.success) {
        logger.info(`AI action completed: ${action.description}`, {
          observations: result.observations
        });

        // Update AI with new observations
        await this.updateAIWithObservations(result.observations);
      } else {
        logger.warn(`AI action failed: ${action.description}`, {
          error: result.error
        });

        // Let AI replan based on failure
        await this.requestAIReplan(action, result.error);
      }

      // Log action to blockchain for verification
      await this.logActionToBlockchain(action, result);
    }

    logger.info('Room exploration completed');
  }

  private async runObjectRetrieval(): Promise<void> {
    logger.info('Starting AI-powered object retrieval task...');

    const retrievalPrompt = `
      There is a small box located approximately 3 meters in front of the robot.
      Your task is to:
      1. Navigate to the box
      2. Pick up the box carefully
      3. Carry the box to a target location 2 meters to the right
      4. Place the box down gently

      Robot capabilities: walking, turning, bending, gripping with 20N force
      Box properties: 0.2m x 0.2m x 0.1m, estimated weight: 0.5kg

      Generate a detailed action plan for this retrieval task.
    `;

    const retrievalPlan = await this.aiEngine.generateActionPlan(retrievalPrompt);
    
    logger.info('AI generated retrieval plan', {
      actionCount: retrievalPlan.actions.length,
      complexity: retrievalPlan.complexity
    });

    let successfulActions = 0;
    
    for (const action of retrievalPlan.actions) {
      if (!this.isRunning) break;

      logger.info(`Executing retrieval action: ${action.description}`);
      
      const result = await this.executeAIAction(action);
      
      if (result.success) {
        successfulActions++;
        logger.info(`Retrieval action completed: ${action.description}`);

        // Special handling for grip actions
        if (action.type === 'grip') {
          await this.verifyGripSecurity();
        }
      } else {
        logger.error(`Retrieval action failed: ${action.description}`, {
          error: result.error
        });

        // Implement recovery strategy
        await this.executeRecoveryStrategy(action, result.error);
      }
    }

    const successRate = (successfulActions / retrievalPlan.actions.length) * 100;
    logger.info('Object retrieval task completed', {
      successRate: `${successRate.toFixed(1)}%`,
      successfulActions,
      totalActions: retrievalPlan.actions.length
    });
  }

  private async runInteractiveTask(): Promise<void> {
    logger.info('Starting interactive AI task...');

    // Simulate dynamic environment changes
    this.simulateEnvironmentChanges();

    const interactivePrompt = `
      You are controlling a robot in a dynamic environment. 
      New obstacles may appear, and task requirements may change.
      
      Initial task: Move to the center of the room and perform a visual inspection.
      However, be prepared to adapt to changes in the environment.

      Current sensors: camera, lidar, IMU
      Adaptation capabilities: replanning, obstacle avoidance, emergency stop

      Generate an adaptive action plan.
    `;

    const interactivePlan = await this.aiEngine.generateActionPlan(interactivePrompt);
    
    logger.info('AI generated adaptive plan', {
      actionCount: interactivePlan.actions.length,
      adaptabilityScore: interactivePlan.adaptability
    });

    // Execute with real-time monitoring
    for (const action of interactivePlan.actions) {
      if (!this.isRunning) break;

      // Check for environment changes before each action
      const environmentChanged = await this.checkEnvironmentChanges();
      if (environmentChanged) {
        logger.info('Environment changed, requesting AI replan...');
        const newPlan = await this.requestAIReplan(action, 'environment_changed');
        if (newPlan) {
          // Execute the new plan instead
          await this.executeAIAction(newPlan.actions[0]);
          continue;
        }
      }

      const result = await this.executeAIAction(action);
      
      if (!result.success) {
        logger.warn('Action failed in dynamic environment', {
          action: action.description,
          error: result.error
        });

        // Implement adaptive recovery
        await this.adaptiveRecovery(action, result.error);
      }
    }

    logger.info('Interactive task completed');
  }

  private async executeAIAction(action: any): Promise<any> {
    logger.debug('Executing AI action', { action });

    try {
      // Convert AI action to robot command
      const command = this.convertAIActionToCommand(action);
      
      // Verify action safety
      const safetyCheck = await this.performSafetyCheck(action);
      if (!safetyCheck.safe) {
        throw new Error(`Safety violation: ${safetyCheck.violations.join(', ')}`);
      }

      // Execute command
      const result = await this.humanoid.executeCommand(command);
      
      // Collect post-execution observations
      const observations = await this.collectObservations();
      
      return {
        success: true,
        result,
        observations,
        safetyCheck
      };

    } catch (error) {
      logger.error('AI action execution failed', error as Error);
      return {
        success: false,
        error: (error as Error).message,
        observations: await this.collectObservations()
      };
    }
  }

  private convertAIActionToCommand(action: any): any {
    // Map AI action types to robot commands
    const actionMap: { [key: string]: any } = {
      'move_forward': {
        type: 'MOVEMENT',
        params: { direction: 'forward', distance: action.distance || 1.0, speed: 0.2 }
      },
      'move_backward': {
        type: 'MOVEMENT', 
        params: { direction: 'backward', distance: action.distance || 0.5, speed: 0.15 }
      },
      'turn_left': {
        type: 'MOVEMENT',
        params: { rotation: -90 } // degrees
      },
      'turn_right': {
        type: 'MOVEMENT',
        params: { rotation: 90 }
      },
      'grip': {
        type: 'MANIPULATION',
        params: { action: 'grip', target: 'right_arm', force: action.force || 15.0 }
      },
      'release': {
        type: 'MANIPULATION',
        params: { action: 'release', target: 'right_arm' }
      },
      'look_around': {
        type: 'SYSTEM',
        params: { action: 'pan_tilt', pan: 90, tilt: 30 }
      }
    };

    const commandTemplate = actionMap[action.type] || {
      type: 'SYSTEM',
      params: action.parameters
    };

    return {
      id: `ai_${action.type}_${Date.now()}`,
      ...commandTemplate,
      priority: action.priority || 50,
      timeout: action.timeout || 10000,
      createdAt: new Date()
    };
  }

  private async performSafetyCheck(action: any): Promise<any> {
    const violations: string[] = [];
    const state = this.humanoid.getState();

    // Check battery level
    if (state.battery < 10) {
      violations.push('low_battery');
    }

    // Check temperature
    if (state.temperature > 50) {
      violations.push('high_temperature');
    }

    // Check for obstacles in path (simulated)
    if (action.type.includes('move') && this.hasObstaclesInPath(action)) {
      violations.push('obstacle_in_path');
    }

    // Check force limits for manipulation
    if (action.type === 'grip' && (action.force || 15) > 25) {
      violations.push('excessive_force');
    }

    return {
      safe: violations.length === 0,
      violations,
      timestamp: new Date()
    };
  }

  private async collectObservations(): Promise<any> {
    const state = this.humanoid.getState();
    
    // Simulate sensor data collection
    return {
      robot_pose: state.pose,
      battery_level: state.battery,
      joint_states: Array.from(state.joints.entries()),
      timestamp: new Date(),
      environment: {
        objects: this.simulateObjectDetection(),
        hazards: this.simulateHazardDetection(),
        changes: this.simulateEnvironmentChanges()
      }
    };
  }

  private async updateAIWithObservations(observations: any): Promise<void> {
    // Update AI context with new observations
    await this.aiEngine.updateContext({
      observations,
      robotState: this.humanoid.getState(),
      timestamp: new Date()
    });
  }

  private async requestAIReplan(failedAction: any, reason: string): Promise<any> {
    const replanPrompt = `
      Previous action failed: ${failedAction.description}
      Failure reason: ${reason}
      Current robot state: ${JSON.stringify(this.humanoid.getState())}
      
      Generate a recovery plan or alternative approach.
    `;

    return await this.aiEngine.generateActionPlan(replanPrompt);
  }

  private async verifyGripSecurity(): Promise<void> {
    logger.info('Verifying grip security...');
    
    // Simulate grip verification
    await this.delay(1000);
    
    const gripSecure = Math.random() > 0.1; // 90% success rate in simulation
    if (!gripSecure) {
      logger.warn('Grip verification failed, retrying...');
      await this.executeRecoveryStrategy({ type: 'regrip' }, 'grip_failed');
    } else {
      logger.info('Grip verified secure');
    }
  }

  private async executeRecoveryStrategy(failedAction: any, error: string): Promise<void> {
    logger.info(`Executing recovery strategy for: ${failedAction.type}`);
    
    const recoveryActions = await this.generateRecoveryActions(failedAction, error);
    
    for (const recoveryAction of recoveryActions) {
      const result = await this.executeAIAction(recoveryAction);
      if (result.success) {
        logger.info('Recovery action successful');
        break;
      }
    }
  }

  private async adaptiveRecovery(failedAction: any, error: string): Promise<void> {
    logger.info(`Executing adaptive recovery for: ${failedAction.type}`);
    
    // Use AI to generate adaptive recovery strategy
    const adaptivePlan = await this.requestAIReplan(failedAction, error);
    
    if (adaptivePlan && adaptivePlan.actions.length > 0) {
      await this.executeAIAction(adaptivePlan.actions[0]);
    } else {
      // Fallback to basic recovery
      await this.executeRecoveryStrategy(failedAction, error);
    }
  }

  private async logActionToBlockchain(action: any, result: any): Promise<void> {
    try {
      const blockchainAction = {
        robotId: 'ai_humanoid_001',
        action: action.type,
        parameters: action.parameters,
        result: result.success ? 'completed' : 'failed',
        timestamp: new Date(),
        verification: {
          safety_checks: 'passed',
          performance: result.result?.performance || 'unknown'
        }
      };

      const txResult = await this.solanaClient.logRobotAction(blockchainAction);
      
      logger.info('Action logged to blockchain', {
        transactionHash: txResult.transactionHash,
        actionId: blockchainAction.action
      });

    } catch (error) {
      logger.error('Failed to log action to blockchain', error as Error);
    }
  }

  // Simulation helper methods
  private hasObstaclesInPath(action: any): boolean {
    // Simulate obstacle detection
    return Math.random() < 0.2; // 20% chance of obstacle
  }

  private simulateObjectDetection(): any[] {
    return [
      {
        id: 'object_1',
        type: 'chair',
        pose: { x: 1.5, y: 0.5, z: 0, roll: 0, pitch: 0, yaw: 0 },
        confidence: 0.85
      },
      {
        id: 'object_2', 
        type: 'table',
        pose: { x: 2.0, y: -1.0, z: 0, roll: 0, pitch: 0, yaw: 0 },
        confidence: 0.92
      }
    ];
  }

  private simulateHazardDetection(): any[] {
    return [
      {
        id: 'hazard_1',
        type: 'slippery_surface',
        location: { x: 0.8, y: 0.3, z: 0 },
        radius: 0.5,
        severity: 'medium'
      }
    ];
  }

  private simulateEnvironmentChanges(): boolean {
    // 10% chance of environment change
    return Math.random() < 0.1;
  }

  private async checkEnvironmentChanges(): Promise<boolean> {
    await this.delay(500);
    return this.simulateEnvironmentChanges();
  }

  private async generateRecoveryActions(failedAction: any, error: string): Promise<any[]> {
    // Generate basic recovery actions based on failure type
    const recoveryMap: { [key: string]: any[] } = {
      'grip_failed': [
        { type: 'regrip', parameters: { force: 18.0 } },
        { type: 'adjust_grip', parameters: { position: 'alternate' } }
      ],
      'obstacle_in_path': [
        { type: 'move_around_obstacle', parameters: { direction: 'left' } },
        { type: 'move_around_obstacle', parameters: { direction: 'right' } }
      ],
      'navigation_failed': [
        { type: 'recalibrate_position' },
        { type: 'request_human_assistance' }
      ]
    };

    return recoveryMap[error] || [{ type: 'retry', parameters: failedAction.parameters }];
  }

  private async handleError(error: Error): Promise<void> {
    logger.error('AI automation error handler activated', error);
    
    try {
      // Safe the robot
      await this.humanoid.emergencyStop();
      await this.humanoid.stand();
      
      // Notify AI system of failure
      await this.aiEngine.handleError(error);
      
      logger.info('Error handling completed');
    } catch (safeError) {
      logger.error('Error during error handling', safeError as Error);
    }
  }

  private async cleanup(): Promise<void> {
    this.isRunning = false;
    
    logger.info('Cleaning up AI automation system...');
    
    try {
      await this.humanoid.stand();
      await this.humanoid.disconnect();
      await this.robotController.disconnect();
      await this.solanaClient.disconnect();
      await this.aiEngine.shutdown();
      
      logger.info('AI automation system cleanup completed');
    } catch (error) {
      logger.error('Error during cleanup', error as Error);
    }
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// Main execution
if (require.main === module) {
  const aiAutomation = new AIAutomation();
  
  aiAutomation.start().catch(error => {
    logger.error('Fatal error in AI automation', error);
    process.exit(1);
  });

  // Handle graceful shutdown
  process.on('SIGINT', async () => {
    logger.info('Received shutdown signal for AI automation...');
    await aiAutomation.cleanup();
    process.exit(0);
  });
}

export { AIAutomation };