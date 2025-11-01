/**
 * Example: Running a Humanoid Robot with SynapseRobo SDK
 * 
 * This example demonstrates:
 * - Connecting to a humanoid robot
 * - Executing movement sequences
 * - Real-time state monitoring
 * - Safety system integration
 * - Error handling and recovery
 */

import { RobotController } from '../src/core/robotController';
import { Humanoid } from '../src/robots/humanoid';
import { HardwareAdapter } from '../src/adapters/hardwareAdapter';
import { SimulationAdapter } from '../src/adapters/simulationAdapter';
import { logger } from '../src/utils/logger';
import { COMMAND_TYPES, ROBOT_TYPES } from '../src/constants';

class RobotRunner {
  private robotController: RobotController;
  private humanoid: Humanoid;
  private isRunning: boolean = false;

  constructor() {
    this.initializeSystem();
  }

  private initializeSystem(): void {
    logger.info('Initializing SynapseRobo Robot System...');

    // Initialize hardware adapter with simulation for this example
    const simulationAdapter = new SimulationAdapter();
    const hardwareAdapter = new HardwareAdapter(simulationAdapter);

    // Create humanoid robot instance
    this.humanoid = new Humanoid(hardwareAdapter, {
      height: 1.75,
      armLength: 0.7,
      legLength: 0.85,
      maxStepHeight: 0.15,
      balanceEnabled: true
    });

    // Initialize robot controller
    this.robotController = new RobotController({
      simulation: true,
      timeout: 30000,
      maxVelocity: 0.4,
      safetyEnabled: true
    });

    logger.info('Robot system initialized successfully');
  }

  public async start(): Promise<void> {
    try {
      this.isRunning = true;
      logger.info('Starting robot demonstration...');

      // Connect to robot
      await this.connectToRobot();
      
      // Run demonstration sequence
      await this.runDemonstrationSequence();

      logger.info('Robot demonstration completed successfully');

    } catch (error) {
      logger.error('Robot demonstration failed', error as Error);
      await this.handleError(error as Error);
    } finally {
      await this.cleanup();
    }
  }

  private async connectToRobot(): Promise<void> {
    logger.info('Connecting to humanoid robot...');
    
    await this.humanoid.connect();
    await this.robotController.addRobot('demo_humanoid', this.humanoid);

    const state = this.humanoid.getState();
    logger.info('Robot connected successfully', {
      battery: state.battery,
      jointCount: state.joints.size,
      pose: state.pose
    });

    // Start state monitoring
    this.startStateMonitoring();
  }

  private startStateMonitoring(): void {
    const monitorInterval = setInterval(() => {
      if (!this.isRunning) {
        clearInterval(monitorInterval);
        return;
      }

      const state = this.humanoid.getState();
      logger.metric('robot_state', 1, {
        battery: state.battery.toString(),
        temperature: state.temperature.toString(),
        connected: state.connected.toString()
      });

      // Log warning if battery is low
      if (state.battery < 20) {
        logger.warn('Robot battery level critical', { battery: state.battery });
      }

    }, 5000); // Monitor every 5 seconds
  }

  private async runDemonstrationSequence(): Promise<void> {
    logger.info('Starting demonstration sequence...');

    // Sequence 1: Basic movements
    await this.executeBasicMovements();

    // Sequence 2: Complex maneuvers
    await this.executeComplexManeuvers();

    // Sequence 3: Object interaction
    await this.executeObjectInteraction();

    // Sequence 4: Emergency response test
    await this.testEmergencyResponse();
  }

  private async executeBasicMovements(): Promise<void> {
    logger.info('Executing basic movement sequence...');

    const movements = [
      { direction: 'forward', distance: 1.0, speed: 0.2, description: 'Walk forward 1 meter' },
      { direction: 'backward', distance: 0.5, speed: 0.15, description: 'Walk backward 0.5 meters' },
      { direction: 'left', distance: 0.8, speed: 0.18, description: 'Side step left' },
      { direction: 'right', distance: 0.8, speed: 0.18, description: 'Side step right' }
    ];

    for (const move of movements) {
      logger.info(`Executing: ${move.description}`);
      
      const command = {
        id: `move_${Date.now()}`,
        type: 'MOVEMENT' as const,
        priority: 50,
        params: {
          direction: move.direction,
          distance: move.distance,
          speed: move.speed
        },
        timeout: 10000,
        createdAt: new Date()
      };

      const result = await this.humanoid.executeCommand(command);
      
      if (result.success) {
        logger.info(`Movement completed: ${move.description}`, {
          distance: result.distance,
          duration: result.duration
        });
      } else {
        throw new Error(`Movement failed: ${move.description} - ${result.error}`);
      }

      // Brief pause between movements
      await this.delay(1000);
    }
  }

  private async executeComplexManeuvers(): Promise<void> {
    logger.info('Executing complex maneuvers...');

    // Turn in place
    logger.info('Executing: Turn 90 degrees right');
    const turnResult = await this.humanoid.rotate(90);
    if (!turnResult.success) {
      throw new Error('Turn maneuver failed');
    }

    // Wave gesture
    logger.info('Executing: Wave gesture');
    const waveResult = await this.humanoid.wave('right');
    if (!waveResult.success) {
      throw new Error('Wave gesture failed');
    }

    // Balance test - stand on one leg
    logger.info('Executing: Balance test - stand on right leg');
    const balanceResult = await this.humanoid.standOnOneLeg('right');
    if (!balanceResult.success) {
      logger.warn('Balance test completed with reduced stability');
    }

    // Return to normal stance
    await this.humanoid.stand();
  }

  private async executeObjectInteraction(): Promise<void> {
    logger.info('Executing object interaction sequence...');

    // Simulate picking up an object
    logger.info('Picking up object with right arm');
    
    const pickupSequence = [
      {
        type: 'MANIPULATION' as const,
        params: { action: 'grip', target: 'right_arm', force: 18.0 }
      },
      {
        type: 'MOVEMENT' as const,
        params: { 
          targetJointPositions: { 
            shoulder_pitch: 0.3,
            elbow: 1.1,
            wrist_pitch: -0.2
          }
        }
      },
      {
        type: 'MOVEMENT' as const,
        params: { 
          targetJointPositions: { 
            shoulder_pitch: -0.2,
            elbow: 0.8,
            wrist_pitch: 0.1
          }
        }
      }
    ];

    for (const cmd of pickupSequence) {
      const command = {
        id: `pickup_${Date.now()}`,
        type: cmd.type,
        priority: 50,
        params: cmd.params,
        timeout: 5000,
        createdAt: new Date()
      };

      const result = await this.humanoid.executeCommand(command);
      if (!result.success) {
        throw new Error(`Pickup sequence failed at step: ${cmd.type}`);
      }
      await this.delay(500);
    }

    // Hold object for 2 seconds
    logger.info('Holding object...');
    await this.delay(2000);

    // Release object
    logger.info('Releasing object...');
    const releaseCommand = {
      id: `release_${Date.now()}`,
      type: 'MANIPULATION' as const,
      priority: 50,
      params: { action: 'release', target: 'right_arm' },
      timeout: 3000,
      createdAt: new Date()
    };

    await this.humanoid.executeCommand(releaseCommand);
  }

  private async testEmergencyResponse(): Promise<void> {
    logger.info('Testing emergency response system...');

    // Start a long movement
    const longMoveCommand = {
      id: 'emergency_test_move',
      type: 'MOVEMENT' as const,
      priority: 50,
      params: {
        direction: 'forward',
        distance: 5.0,
        speed: 0.25
      },
      timeout: 30000,
      createdAt: new Date()
    };

    const movePromise = this.humanoid.executeCommand(longMoveCommand);
    
    // Trigger emergency stop after 2 seconds
    setTimeout(() => {
      logger.warn('EMERGENCY STOP TRIGGERED');
      this.humanoid.emergencyStop();
    }, 2000);

    try {
      const result = await movePromise;
      if (!result.success && result.error?.includes('emergency')) {
        logger.info('Emergency stop test completed successfully');
      } else {
        logger.warn('Emergency stop test completed unexpectedly');
      }
    } catch (error) {
      logger.info('Emergency stop intercepted movement as expected');
    }

    // Reset robot state after emergency stop
    await this.humanoid.stand();
  }

  private async handleError(error: Error): Promise<void> {
    logger.error('Handling robot error', error);
    
    // Attempt to safe the robot
    try {
      await this.humanoid.emergencyStop();
      await this.humanoid.stand(); // Return to safe position
    } catch (safeError) {
      logger.error('Failed to safe robot after error', safeError as Error);
    }

    // Log detailed error information
    logger.metric('robot_error', 1, {
      errorType: error.name,
      errorMessage: error.message
    });
  }

  private async cleanup(): Promise<void> {
    this.isRunning = false;
    
    logger.info('Cleaning up robot system...');
    
    try {
      // Return robot to safe state
      await this.humanoid.stand();
      await this.humanoid.disconnect();
      await this.robotController.disconnect();
      
      logger.info('Robot system cleanup completed');
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
  const robotRunner = new RobotRunner();
  
  robotRunner.start().catch(error => {
    logger.error('Fatal error in robot runner', error);
    process.exit(1);
  });

  // Handle graceful shutdown
  process.on('SIGINT', async () => {
    logger.info('Received shutdown signal...');
    await robotRunner.cleanup();
    process.exit(0);
  });
}

export { RobotRunner };