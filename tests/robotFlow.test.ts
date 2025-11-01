import { RobotController } from '../src/core/robotController';
import { Humanoid } from '../src/robots/humanoid';
import { ArmBot } from '../src/robots/armBot';
import { HardwareAdapter } from '../src/adapters/hardwareAdapter';
import { SimulationAdapter } from '../src/adapters/simulationAdapter';
import { logger } from '../src/utils/logger';
import { ROBOT_TYPES, COMMAND_TYPES, ERROR_CODES, SAFETY_LIMITS } from '../src/constants';

describe('Robot Flow Integration Tests', () => {
  let robotController: RobotController;
  let humanoid: Humanoid;
  let armBot: ArmBot;
  let hardwareAdapter: HardwareAdapter;
  let simulationAdapter: SimulationAdapter;

  beforeEach(() => {
    // Setup simulation adapter for testing
    simulationAdapter = new SimulationAdapter();
    hardwareAdapter = new HardwareAdapter(simulationAdapter);
    
    humanoid = new Humanoid(hardwareAdapter, {
      height: 1.8,
      armLength: 0.7,
      legLength: 0.9,
      maxStepHeight: 0.2,
      balanceEnabled: true
    });

    armBot = new ArmBot(hardwareAdapter, {
      dof: 6,
      reach: 1.2,
      payload: 5.0,
      precision: 0.001
    });

    robotController = new RobotController({
      simulation: true,
      timeout: 30000,
      maxVelocity: 0.5,
      safetyEnabled: true
    });
  });

  afterEach(async () => {
    await robotController.disconnect();
    await humanoid.disconnect();
    await armBot.disconnect();
  });

  describe('Humanoid Robot Tests', () => {
    test('should connect and initialize humanoid robot', async () => {
      await humanoid.connect();
      expect(humanoid.isConnected()).toBe(true);
      
      const state = humanoid.getState();
      expect(state.connected).toBe(true);
      expect(state.battery).toBeGreaterThan(0);
      expect(state.joints.size).toBeGreaterThan(0);
    });

    test('should execute walking sequence', async () => {
      await humanoid.connect();
      
      const walkCommand = {
        id: 'walk_test_1',
        type: 'MOVEMENT' as const,
        priority: 50,
        params: {
          direction: 'forward',
          distance: 1.0,
          speed: 0.3
        },
        timeout: 10000,
        createdAt: new Date()
      };

      const result = await humanoid.executeCommand(walkCommand);
      expect(result.success).toBe(true);
      expect(result.distance).toBeCloseTo(1.0, 1);
    });

    test('should handle obstacle avoidance', async () => {
      await humanoid.connect();
      
      // Simulate obstacle detection
      humanoid.updateEnvironment([
        {
          id: 'obstacle_1',
          type: 'box',
          pose: { x: 0.5, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
          dimensions: { width: 0.3, height: 0.5, depth: 0.3 },
          properties: { avoidable: true }
        }
      ]);

      const walkCommand = {
        id: 'walk_avoid_1',
        type: 'MOVEMENT' as const,
        priority: 50,
        params: {
          direction: 'forward',
          distance: 2.0,
          speed: 0.2
        },
        timeout: 15000,
        createdAt: new Date()
      };

      const result = await humanoid.executeCommand(walkCommand);
      expect(result.success).toBe(true);
      expect(result.obstaclesAvoided).toBeGreaterThan(0);
    });

    test('should maintain balance during movement', async () => {
      await humanoid.connect();
      
      const balanceTest = async () => {
        const initialState = humanoid.getState();
        
        // Execute series of movements that challenge balance
        const movements = [
          { direction: 'left', distance: 0.5, speed: 0.2 },
          { direction: 'right', distance: 1.0, speed: 0.3 },
          { direction: 'forward', distance: 0.8, speed: 0.25 }
        ];

        for (const move of movements) {
          const command = {
            id: `balance_${Date.now()}`,
            type: 'MOVEMENT' as const,
            priority: 75,
            params: move,
            timeout: 8000,
            createdAt: new Date()
          };

          const result = await humanoid.executeCommand(command);
          expect(result.success).toBe(true);
          
          // Check that robot remains stable
          const state = humanoid.getState();
          expect(state.pose.roll).toBeLessThan(0.3); // Less than 0.3 rad tilt
          expect(state.pose.pitch).toBeLessThan(0.3);
        }
      };

      await expect(balanceTest()).resolves.not.toThrow();
    });

    test('should execute complex manipulation sequence', async () => {
      await humanoid.connect();
      
      const sequence = [
        {
          type: 'MANIPULATION' as const,
          params: { action: 'grip', target: 'right_arm', force: 15.0 }
        },
        {
          type: 'MOVEMENT' as const,
          params: { targetJointPositions: { shoulder_pitch: 0.5, elbow: 1.2 } }
        },
        {
          type: 'MANIPULATION' as const,
          params: { action: 'release', target: 'right_arm' }
        }
      ];

      for (const cmd of sequence) {
        const command = {
          id: `manip_${Date.now()}_${cmd.type}`,
          type: cmd.type,
          priority: 50,
          params: cmd.params,
          timeout: 5000,
          createdAt: new Date()
        };

        const result = await humanoid.executeCommand(command);
        expect(result.success).toBe(true);
      }
    });
  });

  describe('ArmBot Robot Tests', () => {
    test('should perform precise positioning', async () => {
      await armBot.connect();
      
      const targetPose = {
        x: 0.5,
        y: 0.3,
        z: 0.2,
        roll: 0.1,
        pitch: -0.2,
        yaw: 0.3
      };

      const command = {
        id: 'precise_move_1',
        type: 'MOVEMENT' as const,
        priority: 50,
        params: { targetPose, velocity: 0.1 },
        timeout: 8000,
        createdAt: new Date()
      };

      const result = await armBot.executeCommand(command);
      expect(result.success).toBe(true);
      
      const finalPose = armBot.getState().pose;
      expect(finalPose.x).toBeCloseTo(targetPose.x, 2);
      expect(finalPose.y).toBeCloseTo(targetPose.y, 2);
      expect(finalPose.z).toBeCloseTo(targetPose.z, 2);
    });

    test('should handle force-controlled gripping', async () => {
      await armBot.connect();
      
      const gripCommand = {
        id: 'force_grip_1',
        type: 'MANIPULATION' as const,
        priority: 50,
        params: {
          action: 'grip',
          target: 'gripper',
          force: 25.0,
          speed: 0.05
        },
        timeout: 5000,
        createdAt: new Date()
      };

      const result = await armBot.executeCommand(gripCommand);
      expect(result.success).toBe(true);
      expect(result.forceApplied).toBeCloseTo(25.0, 1);
    });

    test('should follow trajectory accurately', async () => {
      await armBot.connect();
      
      const waypoints = [
        { x: 0.3, y: 0.2, z: 0.1, roll: 0, pitch: 0, yaw: 0 },
        { x: 0.4, y: 0.3, z: 0.2, roll: 0.1, pitch: 0, yaw: 0.1 },
        { x: 0.5, y: 0.2, z: 0.3, roll: 0, pitch: -0.1, yaw: 0 }
      ];

      let totalError = 0;
      
      for (const waypoint of waypoints) {
        const command = {
          id: `traj_${Date.now()}`,
          type: 'MOVEMENT' as const,
          priority: 50,
          params: { targetPose: waypoint, velocity: 0.2 },
          timeout: 4000,
          createdAt: new Date()
        };

        const result = await armBot.executeCommand(command);
        expect(result.success).toBe(true);
        
        const finalPose = armBot.getState().pose;
        const error = Math.sqrt(
          Math.pow(finalPose.x - waypoint.x, 2) +
          Math.pow(finalPose.y - waypoint.y, 2) +
          Math.pow(finalPose.z - waypoint.z, 2)
        );
        totalError += error;
      }

      const averageError = totalError / waypoints.length;
      expect(averageError).toBeLessThan(0.01); // Less than 1cm average error
    });
  });

  describe('Safety System Tests', () => {
    test('should enforce joint limits', async () => {
      await humanoid.connect();
      
      const dangerousCommand = {
        id: 'dangerous_move',
        type: 'MOVEMENT' as const,
        priority: 50,
        params: {
          targetJointPositions: {
            shoulder_pitch: 5.0, // Beyond safe limit
            elbow: -3.0
          }
        },
        timeout: 5000,
        createdAt: new Date()
      };

      await expect(humanoid.executeCommand(dangerousCommand))
        .rejects
        .toMatchObject({
          code: 'SAFETY_VIOLATION'
        });
    });

    test('should detect and handle collisions', async () => {
      await armBot.connect();
      
      // Simulate collision during movement
      simulationAdapter.simulateCollision(true);
      
      const command = {
        id: 'collision_test',
        type: 'MOVEMENT' as const,
        priority: 50,
        params: {
          targetPose: { x: 0.6, y: 0.4, z: 0.3, roll: 0, pitch: 0, yaw: 0 }
        },
        timeout: 5000,
        createdAt: new Date()
      };

      await expect(armBot.executeCommand(command))
        .rejects
        .toMatchObject({
          code: 'SAFETY_VIOLATION'
        });

      simulationAdapter.simulateCollision(false);
    });

    test('should handle emergency stop', async () => {
      await humanoid.connect();
      
      // Start a long movement
      const longCommand = {
        id: 'long_move',
        type: 'MOVEMENT' as const,
        priority: 50,
        params: {
          direction: 'forward',
          distance: 10.0,
          speed: 0.2
        },
        timeout: 60000,
        createdAt: new Date()
      };

      const movePromise = humanoid.executeCommand(longCommand);
      
      // Emergency stop after 1 second
      setTimeout(() => {
        humanoid.emergencyStop();
      }, 1000);

      const result = await movePromise;
      expect(result.success).toBe(false);
      expect(result.error).toContain('emergency');
    });

    test('should monitor temperature limits', async () => {
      await armBot.connect();
      
      // Simulate overheating
      simulationAdapter.setJointTemperature('elbow', SAFETY_LIMITS.MAX_TEMPERATURE + 5);
      
      const command = {
        id: 'overheat_test',
        type: 'MOVEMENT' as const,
        priority: 50,
        params: {
          targetJointPositions: { shoulder_pitch: 0.5, elbow: 0.8 }
        },
        timeout: 5000,
        createdAt: new Date()
      };

      await expect(armBot.executeCommand(command))
        .rejects
        .toMatchObject({
          code: 'SAFETY_VIOLATION'
        });
    });
  });

  describe('Robot Controller Integration Tests', () => {
    test('should manage multiple robots', async () => {
      await robotController.addRobot('humanoid_1', humanoid);
      await robotController.addRobot('arm_bot_1', armBot);
      
      expect(robotController.getRobotCount()).toBe(2);
      
      const humanoidState = robotController.getRobotState('humanoid_1');
      const armBotState = robotController.getRobotState('arm_bot_1');
      
      expect(humanoidState).toBeDefined();
      expect(armBotState).toBeDefined();
    });

    test('should coordinate multi-robot tasks', async () => {
      await robotController.addRobot('humanoid_1', humanoid);
      await robotController.addRobot('arm_bot_1', armBot);
      
      const coordinatedTask = {
        name: 'assembly_task',
        commands: [
          {
            robotId: 'humanoid_1',
            command: {
              type: 'MANIPULATION' as const,
              params: { action: 'grip', target: 'right_arm', force: 20.0 }
            }
          },
          {
            robotId: 'arm_bot_1',
            command: {
              type: 'MOVEMENT' as const,
              params: { targetPose: { x: 0.4, y: 0.2, z: 0.3, roll: 0, pitch: 0, yaw: 0 } }
            }
          }
        ]
      };

      const results = await robotController.executeCoordinatedTask(coordinatedTask);
      expect(results).toHaveLength(2);
      expect(results.every(r => r.success)).toBe(true);
    });

    test('should handle robot failures gracefully', async () => {
      await robotController.addRobot('humanoid_1', humanoid);
      
      // Simulate robot disconnection during operation
      const movePromise = robotController.executeCommand('humanoid_1', {
        type: 'MOVEMENT' as const,
        params: { direction: 'forward', distance: 2.0 }
      });

      // Simulate unexpected disconnection
      setTimeout(() => {
        humanoid.disconnect();
      }, 500);

      await expect(movePromise)
        .rejects
        .toMatchObject({
          code: 'ROBOT_DISCONNECTED'
        });
    });
  });
});