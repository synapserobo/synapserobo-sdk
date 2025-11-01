/**
 * SynapseRobo SDK - Main Entry Point
 * 
 * Production-grade SDK for Robotics + AI + Solana Execution Network
 * 
 * @packageDocumentation
 * @module synapse-sdk
 * @version 0.1.0
 * @license MIT
 * @copyright SynapseRobo Engineering 2025
 */

// Core SDK Components
export { SolanaClient } from './core/solanaClient';
export { AIEngine } from './core/aiEngine';
export { RobotController } from './core/robotController';
export { TaskManager } from './core/taskManager';
export { Pipeline } from './core/pipeline';

// Robot Implementations
export { Humanoid } from './robots/humanoid';
export { ArmBot } from './robots/armBot';

// Hardware Adapters
export { HardwareAdapter } from './adapters/hardwareAdapter';
export { SimulationAdapter } from './adapters/simulationAdapter';

// Utilities
export { logger, Logger, ChildLogger } from './utils/logger';
export { ConfigManager } from './utils/config';
export * from './utils/types';

// Constants and Types
export * from './constants';

// CLI Entry Point (for programmatic usage)
export { runCLI } from './cli/index';

// SDK Version Information
export const SDK_VERSION = '0.1.0';
export const SDK_NAME = '@synapserobo/sdk';

/**
 * Main SDK initialization function
 * 
 * @example
 * ```typescript
 * import { initializeSDK } from '@synapserobo/sdk';
 * 
 * const sdk = await initializeSDK({
 *   solana: {
 *     network: 'devnet',
 *     commitment: 'confirmed'
 *   },
 *   robot: {
 *     simulation: true,
 *     safetyEnabled: true
 *   },
 *   ai: {
 *     model: 'local-llm',
 *     temperature: 0.7
 *   }
 * });
 * ```
 * 
 * @param config - SDK configuration object
 * @returns Initialized SDK instance with all components
 */
export async function initializeSDK(config?: Partial<import('./utils/types').SDKConfig>): Promise<SDKInstance> {
  const { ConfigManager } = await import('./utils/config');
  const { SolanaClient } = await import('./core/solanaClient');
  const { AIEngine } = await import('./core/aiEngine');
  const { RobotController } = await import('./core/robotController');
  const { TaskManager } = await import('./core/taskManager');
  const { Pipeline } = await import('./core/pipeline');
  const { logger } = await import('./utils/logger');

  try {
    logger.info('Initializing SynapseRobo SDK...', {
      version: SDK_VERSION,
      config: config ? 'custom' : 'default'
    });

    // Initialize configuration
    const configManager = new ConfigManager();
    const fullConfig = configManager.initialize(config);

    // Initialize core components
    const solanaClient = new SolanaClient(fullConfig.solana);
    const aiEngine = new AIEngine(fullConfig.ai);
    const robotController = new RobotController(fullConfig.robot);
    const taskManager = new TaskManager();
    const pipeline = new Pipeline();

    // Connect components
    await solanaClient.connect();
    await aiEngine.initialize();
    await robotController.initialize();

    logger.info('SynapseRobo SDK initialized successfully', {
      components: ['solana', 'ai', 'robot', 'task', 'pipeline'],
      network: fullConfig.solana.network,
      simulation: fullConfig.robot.simulation
    });

    return {
      config: fullConfig,
      solana: solanaClient,
      ai: aiEngine,
      robot: robotController,
      task: taskManager,
      pipeline: pipeline,
      logger: logger,
      
      /**
       * Cleanly shutdown the SDK
       */
      async shutdown(): Promise<void> {
        logger.info('Shutting down SynapseRobo SDK...');
        
        try {
          await robotController.disconnect();
          await solanaClient.disconnect();
          await aiEngine.shutdown();
          
          logger.info('SDK shutdown completed successfully');
        } catch (error) {
          logger.error('Error during SDK shutdown', error as Error);
          throw error;
        }
      },

      /**
       * Get SDK status and health information
       */
      getStatus(): SDKStatus {
        return {
          version: SDK_VERSION,
          initialized: true,
          components: {
            solana: solanaClient.getWalletState(),
            ai: aiEngine.getStatus(),
            robot: robotController.getStatus(),
            task: taskManager.getStatus(),
            pipeline: pipeline.getStatus()
          },
          timestamp: new Date()
        };
      },

      /**
       * Execute a complete AI-to-action pipeline
       */
      async executeAITask(prompt: string, options?: AITaskOptions): Promise<TaskResult> {
        logger.info('Executing AI task', { prompt: prompt.substring(0, 100) + '...' });

        try {
          // Generate AI plan
          const aiResponse = await aiEngine.generateActionPlan(prompt, options?.aiOptions);
          
          // Verify plan safety
          const safetyCheck = await robotController.verifyPlanSafety(aiResponse.actions);
          if (!safetyCheck.safe) {
            throw new Error(`Safety violation: ${safetyCheck.violations.join(', ')}`);
          }

          // Create and execute task
          const task = await taskManager.createTask({
            name: options?.taskName || `AI Task ${Date.now()}`,
            description: `AI-generated task: ${prompt.substring(0, 50)}...`,
            commands: aiResponse.actions.map(action => ({
              id: `ai_action_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
              type: 'MOVEMENT', // This would be mapped from AI action type
              priority: options?.priority || 50,
              params: action.parameters,
              timeout: options?.timeout || 30000,
              createdAt: new Date()
            })),
            priority: options?.priority || 50,
            timeout: options?.timeout || 300000
          });

          // Execute through pipeline
          const result = await pipeline.executeTask(task);
          
          // Log to blockchain if requested
          if (options?.logToBlockchain) {
            await solanaClient.logRobotAction({
              robotId: 'sdk_ai_robot',
              action: 'ai_task_completion',
              parameters: {
                prompt: prompt.substring(0, 200),
                taskId: task.id,
                success: result.success
              },
              timestamp: new Date()
            });
          }

          return result;

        } catch (error) {
          logger.error('AI task execution failed', error as Error);
          throw error;
        }
      },

      /**
       * Register a new robot with the system
       */
      async registerRobot(type: string, config: any): Promise<string> {
        const { Humanoid } = await import('./robots/humanoid');
        const { ArmBot } = await import('./robots/armBot');
        const { HardwareAdapter } = await import('./adapters/hardwareAdapter');
        const { SimulationAdapter } = await import('./adapters/simulationAdapter');

        const hardwareAdapter = new HardwareAdapter(new SimulationAdapter());
        let robot;

        switch (type) {
          case 'humanoid':
            robot = new Humanoid(hardwareAdapter, config);
            break;
          case 'arm_bot':
            robot = new ArmBot(hardwareAdapter, config);
            break;
          default:
            throw new Error(`Unsupported robot type: ${type}`);
        }

        await robot.connect();
        const robotId = await robotController.addRobot(`registered_${type}_${Date.now()}`, robot);
        
        logger.info('Robot registered successfully', { robotId, type });
        return robotId;
      }
    };

  } catch (error) {
    logger.error('Failed to initialize SynapseRobo SDK', error as Error);
    throw error;
  }
}

/**
 * Quick start function for common use cases
 * 
 * @example
 * ```typescript
 * // Quick start with default configuration
 * const { robot, ai } = await quickStart();
 * 
 * // Execute a simple AI command
 * const result = await ai.generateActionPlan('Walk forward and wave');
 * await robot.executeCommands(result.actions);
 * ```
 */
export async function quickStart(options?: QuickStartOptions): Promise<QuickStartInstance> {
  const sdk = await initializeSDK(options?.config);

  return {
    robot: sdk.robot,
    ai: sdk.ai,
    solana: sdk.solana,
    
    /**
     * Execute a simple AI-powered robot command
     */
    async executeCommand(prompt: string): Promise<CommandResult> {
      const taskResult = await sdk.executeAITask(prompt, {
        taskName: `Quick Command: ${prompt.substring(0, 30)}...`,
        priority: 50,
        timeout: 60000,
        logToBlockchain: options?.enableBlockchain || false
      });

      return {
        success: taskResult.success,
        taskId: taskResult.taskId,
        duration: taskResult.duration,
        results: taskResult.results,
        error: taskResult.error
      };
    },

    /**
     * Get system status
     */
    getStatus() {
      return sdk.getStatus();
    },

    /**
     * Cleanup resources
     */
    async shutdown() {
      await sdk.shutdown();
    }
  };
}

// Type definitions for the SDK
export interface SDKInstance {
  config: import('./utils/types').SDKConfig;
  solana: import('./core/solanaClient').SolanaClient;
  ai: import('./core/aiEngine').AIEngine;
  robot: import('./core/robotController').RobotController;
  task: import('./core/taskManager').TaskManager;
  pipeline: import('./core/pipeline').Pipeline;
  logger: import('./utils/logger').Logger;
  shutdown(): Promise<void>;
  getStatus(): SDKStatus;
  executeAITask(prompt: string, options?: AITaskOptions): Promise<TaskResult>;
  registerRobot(type: string, config: any): Promise<string>;
}

export interface QuickStartInstance {
  robot: import('./core/robotController').RobotController;
  ai: import('./core/aiEngine').AIEngine;
  solana: import('./core/solanaClient').SolanaClient;
  executeCommand(prompt: string): Promise<CommandResult>;
  getStatus(): SDKStatus;
  shutdown(): Promise<void>;
}

export interface SDKStatus {
  version: string;
  initialized: boolean;
  components: {
    solana: any;
    ai: any;
    robot: any;
    task: any;
    pipeline: any;
  };
  timestamp: Date;
}

export interface AITaskOptions {
  taskName?: string;
  priority?: number;
  timeout?: number;
  logToBlockchain?: boolean;
  aiOptions?: any;
}

export interface QuickStartOptions {
  config?: Partial<import('./utils/types').SDKConfig>;
  enableBlockchain?: boolean;
}

export interface TaskResult {
  success: boolean;
  taskId: string;
  duration: number;
  results: any[];
  error?: string;
}

export interface CommandResult {
  success: boolean;
  taskId: string;
  duration: number;
  results: any[];
  error?: string;
}

// Default export for convenience
export default initializeSDK;

// Global error handler for uncaught exceptions
process.on('uncaughtException', (error) => {
  console.error('Uncaught Exception in SynapseRobo SDK:', error);
  // In production, this would log to telemetry and potentially restart services
});

process.on('unhandledRejection', (reason, promise) => {
  console.error('Unhandled Rejection in SynapseRobo SDK at:', promise, 'reason:', reason);
  // In production, this would log to telemetry
});