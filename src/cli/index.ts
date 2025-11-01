
/**
 * SynapseRobo CLI - Command Line Interface for Robotics + AI + Solana Execution
 * 
 * Production-grade CLI for controlling robots, managing AI tasks, and handling
 * blockchain transactions through an intuitive command-line interface.
 */

import { Command } from 'commander';
import inquirer from 'inquirer';
import chalk from 'chalk';
import { logger } from '../utils/logger';
import { initializeSDK, quickStart } from '../index';
import { Humanoid } from '../robots/humanoid';
import { ArmBot } from '../robots/armBot';
import { HardwareAdapter } from '../adapters/hardwareAdapter';
import { SimulationAdapter } from '../adapters/simulationAdapter';

class SynapseCLI {
  private program: Command;
  private sdk: any = null;
  private currentRobot: any = null;

  constructor() {
    this.program = new Command();
    this.setupCLI();
  }

  private setupCLI(): void {
    this.program
      .name('synapse')
      .description('SynapseRobo CLI - Robotics + AI + Solana Execution Network')
      .version('0.1.0')
      .option('--verbose', 'Enable verbose logging')
      .option('--simulation', 'Run in simulation mode')
      .option('--network <network>', 'Solana network (mainnet, devnet, testnet)', 'devnet');

    this.setupRobotCommands();
    this.setupAICommands();
    this.setupWalletCommands();
    this.setupTaskCommands();
    this.setupSystemCommands();

    // Add help command
    this.program.addHelpText('after', `
Examples:
  ${chalk.green('synapse run humanoid')}           Run humanoid robot in simulation
  ${chalk.green('synapse simulate arm')}           Simulate robotic arm operations
  ${chalk.green('synapse wallet connect')}         Connect to Solana wallet
  ${chalk.green('synapse wallet status')}          Check wallet status and balance
  ${chalk.green('synapse task queue')}             Show current task queue
  ${chalk.green('synapse ai plan "walk forward"')} Generate AI plan for movement

Environment Variables:
  SOLANA_NETWORK    Solana network to connect to
  ROBOT_SIMULATION  Run in simulation mode (true/false)
  LOG_LEVEL         Logging level (0-4)
    `);
  }

  private setupRobotCommands(): void {
    const robot = this.program.command('robot')
      .description('Robot control commands');

    robot.command('run <type>')
      .description('Run a robot (humanoid, arm)')
      .option('--config <path>', 'Robot configuration file')
      .option('--simulation', 'Run in simulation mode')
      .action(async (type, options) => {
        await this.runRobot(type, options);
      });

    robot.command('status')
      .description('Get robot status')
      .action(async () => {
        await this.getRobotStatus();
      });

    robot.command('move <direction>')
      .description('Move robot in direction (forward, backward, left, right)')
      .option('--distance <number>', 'Distance to move', '1.0')
      .option('--speed <number>', 'Movement speed', '0.3')
      .action(async (direction, options) => {
        await this.moveRobot(direction, options);
      });

    robot.command('grip')
      .description('Grip with robot')
      .option('--force <number>', 'Grip force', '15.0')
      .action(async (options) => {
        await this.gripRobot(options);
      });

    robot.command('release')
      .description('Release grip')
      .action(async () => {
        await this.releaseRobot();
      });

    robot.command('emergency-stop')
      .description('Emergency stop robot')
      .action(async () => {
        await this.emergencyStop();
      });
  }

  private setupAICommands(): void {
    const ai = this.program.command('ai')
      .description('AI and automation commands');

    ai.command('plan <prompt>')
      .description('Generate AI plan from prompt')
      .option('--model <model>', 'AI model to use', 'local-llm')
      .option('--temperature <number>', 'AI temperature', '0.7')
      .action(async (prompt, options) => {
        await this.generateAIPlan(prompt, options);
      });

    ai.command('execute <prompt>')
      .description('Execute AI-generated plan')
      .option('--confirm', 'Confirm before execution')
      .action(async (prompt, options) => {
        await this.executeAIPlan(prompt, options);
      });

    ai.command('automate')
      .description('Start AI automation session')
      .action(async () => {
        await this.startAIAutomation();
      });
  }

  private setupWalletCommands(): void {
    const wallet = this.program.command('wallet')
      .description('Solana wallet commands');

    wallet.command('connect')
      .description('Connect to Solana wallet')
      .option('--keypair <path>', 'Path to keypair file')
      .action(async (options) => {
        await this.connectWallet(options);
      });

    wallet.command('status')
      .description('Get wallet status')
      .action(async () => {
        await this.getWalletStatus();
      });

    wallet.command('balance')
      .description('Get wallet balance')
      .action(async () => {
        await this.getWalletBalance();
      });

    wallet.command('create-transaction')
      .description('Create robot action transaction')
      .option('--action <action>', 'Robot action')
      .option('--parameters <json>', 'Action parameters')
      .action(async (options) => {
        await this.createTransaction(options);
      });
  }

  private setupTaskCommands(): void {
    const task = this.program.command('task')
      .description('Task management commands');

    task.command('queue')
      .description('Show task queue')
      .action(async () => {
        await this.showTaskQueue();
      });

    task.command('create <name>')
      .description('Create new task')
      .option('--description <text>', 'Task description')
      .option('--priority <number>', 'Task priority', '50')
      .action(async (name, options) => {
        await this.createTask(name, options);
      });

    task.command('execute <id>')
      .description('Execute task by ID')
      .action(async (id) => {
        await this.executeTask(id);
      });

    task.command('cancel <id>')
      .description('Cancel task by ID')
      .action(async (id) => {
        await this.cancelTask(id);
      });
  }

  private setupSystemCommands(): void {
    const system = this.program.command('system')
      .description('System management commands');

    system.command('status')
      .description('Get system status')
      .action(async () => {
        await this.getSystemStatus();
      });

    system.command('init')
      .description('Initialize SDK')
      .action(async () => {
        await this.initializeSDK();
      });

    system.command('shutdown')
      .description('Shutdown system')
      .action(async () => {
        await this.shutdownSystem();
      });

    system.command('logs')
      .description('Show recent logs')
      .option('--level <level>', 'Log level', 'info')
      .option('--lines <number>', 'Number of lines', '20')
      .action(async (options) => {
        await this.showLogs(options);
      });
  }

  // Command Implementations
  private async runRobot(type: string, options: any): Promise<void> {
    try {
      console.log(chalk.blue(`🚀 Starting ${type} robot...`));

      // Initialize SDK if not already done
      if (!this.sdk) {
        await this.initializeSDK(options);
      }

      // Create robot based on type
      const hardwareAdapter = new HardwareAdapter(new SimulationAdapter());
      
      switch (type.toLowerCase()) {
        case 'humanoid':
          this.currentRobot = new Humanoid(hardwareAdapter, {
            height: 1.75,
            armLength: 0.7,
            legLength: 0.85,
            maxStepHeight: 0.15,
            balanceEnabled: true
          });
          break;
        case 'arm':
        case 'armbot':
          this.currentRobot = new ArmBot(hardwareAdapter, {
            dof: 6,
            reach: 1.2,
            payload: 5.0,
            precision: 0.001
          });
          break;
        default:
          throw new Error(`Unknown robot type: ${type}`);
      }

      // Connect to robot
      await this.currentRobot.connect();
      
      console.log(chalk.green(`✅ ${type} robot connected successfully`));
      console.log(chalk.blue('🤖 Robot is ready for commands'));

      // If in interactive mode, start command loop
      if (process.stdin.isTTY) {
        await this.startInteractiveMode();
      }

    } catch (error) {
      console.error(chalk.red(`❌ Failed to run robot: ${(error as Error).message}`));
      process.exit(1);
    }
  }

  private async getRobotStatus(): Promise<void> {
    if (!this.currentRobot) {
      console.log(chalk.yellow('⚠️  No robot connected'));
      return;
    }

    try {
      const state = this.currentRobot.getState();
      
      console.log(chalk.blue('🤖 Robot Status:'));
      console.log(`  Connected: ${state.connected ? chalk.green('Yes') : chalk.red('No')}`);
      console.log(`  Battery: ${chalk.blue(state.battery + '%')}`);
      console.log(`  Temperature: ${chalk.blue(state.temperature + '°C')}`);
      console.log(`  Mode: ${chalk.blue(state.mode)}`);
      console.log(`  Position: x=${state.pose.x.toFixed(3)}, y=${state.pose.y.toFixed(3)}, z=${state.pose.z.toFixed(3)}`);
      
      if (state.errors.length > 0) {
        console.log(chalk.red('  Errors:'));
        state.errors.forEach(error => console.log(`    - ${error}`));
      }

    } catch (error) {
      console.error(chalk.red(`❌ Failed to get robot status: ${(error as Error).message}`));
    }
  }

  private async moveRobot(direction: string, options: any): Promise<void> {
    if (!this.currentRobot) {
      console.log(chalk.yellow('⚠️  No robot connected'));
      return;
    }

    try {
      console.log(chalk.blue(`🎯 Moving robot ${direction}...`));

      const command = {
        id: `move_${Date.now()}`,
        type: 'MOVEMENT' as const,
        priority: 50,
        params: {
          direction: direction.toLowerCase(),
          distance: parseFloat(options.distance),
          speed: parseFloat(options.speed)
        },
        timeout: 30000,
        createdAt: new Date()
      };

      const result = await this.currentRobot.executeCommand(command);
      
      if (result.success) {
        console.log(chalk.green(`✅ Movement completed successfully`));
        console.log(`   Distance: ${result.result.actualDistance.toFixed(2)}m`);
        console.log(`   Duration: ${result.duration}ms`);
      } else {
        console.log(chalk.red(`❌ Movement failed: ${result.error}`));
      }

    } catch (error) {
      console.error(chalk.red(`❌ Movement failed: ${(error as Error).message}`));
    }
  }

  private async gripRobot(options: any): Promise<void> {
    if (!this.currentRobot) {
      console.log(chalk.yellow('⚠️  No robot connected'));
      return;
    }

    try {
      console.log(chalk.blue('🤏 Gripping...'));

      const command = {
        id: `grip_${Date.now()}`,
        type: 'MANIPULATION' as const,
        priority: 50,
        params: {
          action: 'grip',
          target: 'gripper',
          force: parseFloat(options.force),
          speed: 0.1
        },
        timeout: 10000,
        createdAt: new Date()
      };

      const result = await this.currentRobot.executeCommand(command);
      
      if (result.success) {
        console.log(chalk.green(`✅ Grip successful`));
        console.log(`   Force: ${result.result.actualForce.toFixed(1)}N`);
        console.log(`   Secure: ${result.result.gripSecure ? 'Yes' : 'No'}`);
      } else {
        console.log(chalk.red(`❌ Grip failed: ${result.error}`));
      }

    } catch (error) {
      console.error(chalk.red(`❌ Grip failed: ${(error as Error).message}`));
    }
  }

  private async releaseRobot(): Promise<void> {
    if (!this.currentRobot) {
      console.log(chalk.yellow('⚠️  No robot connected'));
      return;
    }

    try {
      console.log(chalk.blue('🖐️  Releasing grip...'));

      const command = {
        id: `release_${Date.now()}`,
        type: 'MANIPULATION' as const,
        priority: 50,
        params: {
          action: 'release',
          target: 'gripper',
          speed: 0.1
        },
        timeout: 5000,
        createdAt: new Date()
      };

      const result = await this.currentRobot.executeCommand(command);
      
      if (result.success) {
        console.log(chalk.green(`✅ Release successful`));
      } else {
        console.log(chalk.red(`❌ Release failed: ${result.error}`));
      }

    } catch (error) {
      console.error(chalk.red(`❌ Release failed: ${(error as Error).message}`));
    }
  }

  private async emergencyStop(): Promise<void> {
    if (!this.currentRobot) {
      console.log(chalk.yellow('⚠️  No robot connected'));
      return;
    }

    try {
      console.log(chalk.red('🛑 Emergency stop!'));
      
      this.currentRobot.emergencyStop();
      console.log(chalk.yellow('⚠️  Robot emergency stop activated'));

    } catch (error) {
      console.error(chalk.red(`❌ Emergency stop failed: ${(error as Error).message}`));
    }
  }

  private async generateAIPlan(prompt: string, options: any): Promise<void> {
    try {
      console.log(chalk.blue('🧠 Generating AI plan...'));

      if (!this.sdk) {
        await this.initializeSDK(options);
      }

      const plan = await this.sdk.ai.generateActionPlan(prompt, {
        temperature: parseFloat(options.temperature),
        model: options.model
      });

      console.log(chalk.green('✅ AI Plan Generated:'));
      console.log(`   Actions: ${plan.actions.length}`);
      console.log(`   Confidence: ${(plan.confidence * 100).toFixed(1)}%`);
      console.log(`   Estimated Duration: ${plan.estimatedDuration}s`);
      
      console.log(chalk.blue('\n📋 Action Sequence:'));
      plan.actions.forEach((action: any, index: number) => {
        console.log(`   ${index + 1}. ${action.description}`);
      });

    } catch (error) {
      console.error(chalk.red(`❌ AI plan generation failed: ${(error as Error).message}`));
    }
  }

  private async executeAIPlan(prompt: string, options: any): Promise<void> {
    try {
      console.log(chalk.blue('🚀 Executing AI plan...'));

      if (!this.sdk) {
        await this.initializeSDK(options);
      }

      if (options.confirm) {
        const answers = await inquirer.prompt([
          {
            type: 'confirm',
            name: 'proceed',
            message: 'Are you sure you want to execute this AI plan?',
            default: false
          }
        ]);

        if (!answers.proceed) {
          console.log(chalk.yellow('⚠️  Execution cancelled'));
          return;
        }
      }

      const result = await this.sdk.executeAITask(prompt, {
        logToBlockchain: true
      });

      if (result.success) {
        console.log(chalk.green('✅ AI task executed successfully'));
        console.log(`   Task ID: ${result.taskId}`);
        console.log(`   Duration: ${result.duration}ms`);
        console.log(`   Results: ${result.results.length}`);
      } else {
        console.log(chalk.red(`❌ AI task failed: ${result.error}`));
      }

    } catch (error) {
      console.error(chalk.red(`❌ AI execution failed: ${(error as Error).message}`));
    }
  }

  private async startAIAutomation(): Promise<void> {
    try {
      console.log(chalk.blue('🤖 Starting AI automation session...'));

      if (!this.sdk) {
        await this.initializeSDK();
      }

      // Start interactive automation session
      await this.startInteractiveAutomation();

    } catch (error) {
      console.error(chalk.red(`❌ AI automation failed: ${(error as Error).message}`));
    }
  }

  private async connectWallet(options: any): Promise<void> {
    try {
      console.log(chalk.blue('🔗 Connecting to Solana wallet...'));

      if (!this.sdk) {
        await this.initializeSDK(options);
      }

      const walletState = this.sdk.solana.getWalletState();
      
      console.log(chalk.green('✅ Wallet connected successfully'));
      console.log(`   Address: ${chalk.blue(walletState.publicKey?.toString())}`);
      console.log(`   Network: ${chalk.blue(walletState.network)}`);
      console.log(`   Connected: ${walletState.connected ? chalk.green('Yes') : chalk.red('No')}`);

    } catch (error) {
      console.error(chalk.red(`❌ Wallet connection failed: ${(error as Error).message}`));
    }
  }

  private async getWalletStatus(): Promise<void> {
    if (!this.sdk) {
      console.log(chalk.yellow('⚠️  SDK not initialized'));
      return;
    }

    try {
      const walletState = this.sdk.solana.getWalletState();
      const balance = await this.sdk.solana.getBalance();
      
      console.log(chalk.blue('💰 Wallet Status:'));
      console.log(`   Connected: ${walletState.connected ? chalk.green('Yes') : chalk.red('No')}`);
      console.log(`   Address: ${chalk.blue(walletState.publicKey?.toString() || 'Not connected')}`);
      console.log(`   Network: ${chalk.blue(walletState.network)}`);
      console.log(`   Balance: ${chalk.green(balance + ' SOL')}`);

    } catch (error) {
      console.error(chalk.red(`❌ Failed to get wallet status: ${(error as Error).message}`));
    }
  }

  private async getWalletBalance(): Promise<void> {
    if (!this.sdk) {
      console.log(chalk.yellow('⚠️  SDK not initialized'));
      return;
    }

    try {
      const balance = await this.sdk.solana.getBalance();
      console.log(chalk.green(`💰 Wallet Balance: ${balance} SOL`));

    } catch (error) {
      console.error(chalk.red(`❌ Failed to get wallet balance: ${(error as Error).message}`));
    }
  }

  private async createTransaction(options: any): Promise<void> {
    if (!this.sdk) {
      console.log(chalk.yellow('⚠️  SDK not initialized'));
      return;
    }

    try {
      console.log(chalk.blue('📝 Creating robot action transaction...'));

      const parameters = options.parameters ? JSON.parse(options.parameters) : {};
      
      const transaction = await this.sdk.solana.createRobotActionTransaction({
        robotId: 'cli_robot',
        action: options.action || 'movement',
        parameters,
        timestamp: new Date()
      });

      console.log(chalk.green('✅ Transaction created successfully'));
      console.log(`   Description: ${transaction.description}`);
      console.log(`   Signers: ${transaction.signers.length}`);

    } catch (error) {
      console.error(chalk.red(`❌ Transaction creation failed: ${(error as Error).message}`));
    }
  }

  private async showTaskQueue(): Promise<void> {
    if (!this.sdk) {
      console.log(chalk.yellow('⚠️  SDK not initialized'));
      return;
    }

    try {
      const taskManager = this.sdk.task;
      const queue = taskManager.getQueue();
      
      console.log(chalk.blue('📋 Task Queue:'));
      
      if (queue.length === 0) {
        console.log('   No tasks in queue');
        return;
      }

      queue.forEach((task: any, index: number) => {
        console.log(`   ${index + 1}. ${task.name} (${task.status}) - Priority: ${task.priority}`);
      });

    } catch (error) {
      console.error(chalk.red(`❌ Failed to get task queue: ${(error as Error).message}`));
    }
  }

  private async createTask(name: string, options: any): Promise<void> {
    if (!this.sdk) {
      console.log(chalk.yellow('⚠️  SDK not initialized'));
      return;
    }

    try {
      console.log(chalk.blue('📝 Creating new task...'));

      const task = await this.sdk.task.createTask({
        name,
        description: options.description || 'CLI-created task',
        priority: parseInt(options.priority),
        commands: [], // Would be populated based on task type
        timeout: 300000
      });

      console.log(chalk.green('✅ Task created successfully'));
      console.log(`   ID: ${task.id}`);
      console.log(`   Name: ${task.name}`);
      console.log(`   Priority: ${task.priority}`);

    } catch (error) {
      console.error(chalk.red(`❌ Task creation failed: ${(error as Error).message}`));
    }
  }

  private async executeTask(id: string): Promise<void> {
    if (!this.sdk) {
      console.log(chalk.yellow('⚠️  SDK not initialized'));
      return;
    }

    try {
      console.log(chalk.blue(`🚀 Executing task ${id}...`));

      const result = await this.sdk.pipeline.executeTask(id);
      
      if (result.success) {
        console.log(chalk.green('✅ Task executed successfully'));
        console.log(`   Duration: ${result.duration}ms`);
        console.log(`   Results: ${result.results.length}`);
      } else {
        console.log(chalk.red(`❌ Task execution failed: ${result.error}`));
      }

    } catch (error) {
      console.error(chalk.red(`❌ Task execution failed: ${(error as Error).message}`));
    }
  }

  private async cancelTask(id: string): Promise<void> {
    if (!this.sdk) {
      console.log(chalk.yellow('⚠️  SDK not initialized'));
      return;
    }

    try {
      console.log(chalk.yellow(`⏹️  Cancelling task ${id}...`));

      await this.sdk.task.cancelTask(id);
      console.log(chalk.green('✅ Task cancelled successfully'));

    } catch (error) {
      console.error(chalk.red(`❌ Task cancellation failed: ${(error as Error).message}`));
    }
  }

  private async getSystemStatus(): Promise<void> {
    try {
      if (!this.sdk) {
        console.log(chalk.yellow('⚠️  SDK not initialized - running in limited mode'));
        return;
      }

      const status = this.sdk.getStatus();
      
      console.log(chalk.blue('🖥️  System Status:'));
      console.log(`   Version: ${chalk.blue(status.version)}`);
      console.log(`   Initialized: ${status.initialized ? chalk.green('Yes') : chalk.red('No')}`);
      console.log(`   Timestamp: ${new Date(status.timestamp).toLocaleString()}`);
      
      console.log(chalk.blue('\n🔧 Component Status:'));
      console.log(`   Solana: ${status.components.solana.connected ? chalk.green('Connected') : chalk.red('Disconnected')}`);
      console.log(`   AI: ${status.components.ai.initialized ? chalk.green('Ready') : chalk.red('Not Ready')}`);
      console.log(`   Robot: ${status.components.robot.connected ? chalk.green('Connected') : chalk.red('Disconnected')}`);
      console.log(`   Task Manager: ${status.components.task.ready ? chalk.green('Ready') : chalk.red('Not Ready')}`);

    } catch (error) {
      console.error(chalk.red(`❌ Failed to get system status: ${(error as Error).message}`));
    }
  }

  private async initializeSDK(options: any = {}): Promise<void> {
    try {
      console.log(chalk.blue('⚙️  Initializing SynapseRobo SDK...'));

      const sdkOptions = {
        solana: {
          network: options.network || process.env.SOLANA_NETWORK || 'devnet'
        },
        robot: {
          simulation: options.simulation || process.env.ROBOT_SIMULATION === 'true' || true
        },
        ai: {
          model: 'local-llm',
          temperature: 0.7
        },
        logging: {
          level: options.verbose ? 3 : 1, // DEBUG if verbose, WARN otherwise
          format: 'text'
        }
      };

      this.sdk = await initializeSDK(sdkOptions);
      
      console.log(chalk.green('✅ SDK initialized successfully'));

    } catch (error) {
      console.error(chalk.red(`❌ SDK initialization failed: ${(error as Error).message}`));
      process.exit(1);
    }
  }

  private async shutdownSystem(): Promise<void> {
    try {
      console.log(chalk.blue('🛑 Shutting down system...'));

      if (this.currentRobot) {
        await this.currentRobot.disconnect();
        this.currentRobot = null;
      }

      if (this.sdk) {
        await this.sdk.shutdown();
        this.sdk = null;
      }

      console.log(chalk.green('✅ System shutdown completed'));

    } catch (error) {
      console.error(chalk.red(`❌ System shutdown failed: ${(error as Error).message}`));
    }
  }

  private async showLogs(options: any): Promise<void> {
    try {
      const levelMap: Record<string, number> = {
        error: 0,
        warn: 1,
        info: 2,
        debug: 3,
        trace: 4
      };

      const logs = logger.getLogs({
        level: levelMap[options.level] || 2,
        startTime: new Date(Date.now() - 3600000) // Last hour
      });

      const recentLogs = logs.slice(-parseInt(options.lines));

      console.log(chalk.blue(`📋 Recent logs (${options.level} level):`));
      
      recentLogs.forEach(log => {
        const timestamp = new Date(log.timestamp).toLocaleTimeString();
        const level = log.level.toUpperCase();
        const message = log.message;
        
        let levelColor = chalk.blue;
        if (log.level === 'error') levelColor = chalk.red;
        if (log.level === 'warn') levelColor = chalk.yellow;
        if (log.level === 'info') levelColor = chalk.green;

        console.log(`[${timestamp}] ${levelColor(level)}: ${message}`);
      });

    } catch (error) {
      console.error(chalk.red(`❌ Failed to show logs: ${(error as Error).message}`));
    }
  }

  private async startInteractiveMode(): Promise<void> {
    console.log(chalk.blue('\n🎮 Starting interactive mode...'));
    console.log(chalk.gray('Type "help" for available commands, "exit" to quit'));

    const readline = require('readline');
    const rl = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
      prompt: chalk.blue('🤖> ')
    });

    rl.prompt();

    rl.on('line', async (line) => {
      const input = line.trim().toLowerCase();
      
      switch (input) {
        case 'exit':
        case 'quit':
          console.log(chalk.blue('👋 Goodbye!'));
          rl.close();
          process.exit(0);
          break;
        
        case 'help':
          this.showInteractiveHelp();
          break;
        
        case 'status':
          await this.getRobotStatus();
          break;
        
        case '':
          // Empty input, do nothing
          break;
        
        default:
          if (input.startsWith('move ')) {
            const direction = input.split(' ')[1];
            await this.moveRobot(direction, { distance: '1.0', speed: '0.3' });
          } else if (input === 'grip') {
            await this.gripRobot({ force: '15.0' });
          } else if (input === 'release') {
            await this.releaseRobot();
          } else {
            console.log(chalk.yellow('❓ Unknown command. Type "help" for available commands.'));
          }
      }

      rl.prompt();
    });

    rl.on('close', () => {
      console.log(chalk.blue('👋 Goodbye!'));
      process.exit(0);
    });
  }

  private async startInteractiveAutomation(): Promise<void> {
    console.log(chalk.blue('\n🤖 Starting AI automation session...'));
    console.log(chalk.gray('Describe what you want the robot to do:'));

    const readline = require('readline');
    const rl = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
      prompt: chalk.magenta('AI> ')
    });

    rl.prompt();

    rl.on('line', async (line) => {
      const prompt = line.trim();
      
      if (prompt === 'exit' || prompt === 'quit') {
        console.log(chalk.blue('👋 Ending automation session'));
        rl.close();
        return;
      }

      if (prompt === '') {
        rl.prompt();
        return;
      }

      try {
        console.log(chalk.blue('🧠 Thinking...'));
        
        const result = await this.sdk.executeAITask(prompt, {
          taskName: `Interactive: ${prompt.substring(0, 30)}...`,
          priority: 50,
          timeout: 60000,
          logToBlockchain: false
        });

        if (result.success) {
          console.log(chalk.green('✅ Task completed successfully!'));
          console.log(chalk.gray(`   Duration: ${result.duration}ms`));
          console.log(chalk.gray(`   Results: ${result.results.length} actions`));
        } else {
          console.log(chalk.red(`❌ Task failed: ${result.error}`));
        }

      } catch (error) {
        console.error(chalk.red(`❌ Automation error: ${(error as Error).message}`));
      }

      console.log(chalk.gray('\nWhat should I do next?'));
      rl.prompt();
    });
  }

  private showInteractiveHelp(): void {
    console.log(chalk.blue('\n📖 Available Commands:'));
    console.log('  status                    - Show robot status');
    console.log('  move <direction>          - Move robot (forward, backward, left, right)');
    console.log('  grip                      - Grip with default force');
    console.log('  release                   - Release grip');
    console.log('  emergency-stop            - Emergency stop robot');
    console.log('  help                      - Show this help');
    console.log('  exit                      - Exit interactive mode');
  }

  public async run(): Promise<void> {
    try {
      // Parse command line arguments
      await this.program.parseAsync(process.argv);

      // If no command provided, show help
      if (!process.argv.slice(2).length) {
        this.program.outputHelp();
      }

    } catch (error) {
      console.error(chalk.red(`❌ CLI error: ${(error as Error).message}`));
      process.exit(1);
    }
  }
}

// Export for programmatic usage
export async function runCLI(): Promise<void> {
  const cli = new SynapseCLI();
  await cli.run();
}

// Main execution
if (require.main === module) {
  runCLI().catch(error => {
    console.error(chalk.red(`Fatal CLI error: ${error.message}`));
    process.exit(1);
  });
}