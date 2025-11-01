/**
 * Example: Solana-Powered Micropayments for Robot Services
 * 
 * This example demonstrates:
 * - Integrating Solana payments with robot operations
 * - Pay-per-action business model
 * - Transaction verification and confirmation
 * - Wallet management and security
 * - Multi-signature transactions for high-value operations
 */

import { SolanaClient } from '../src/core/solanaClient';
import { RobotController } from '../src/core/robotController';
import { Humanoid } from '../src/robots/humanoid';
import { HardwareAdapter } from '../src/adapters/hardwareAdapter';
import { SimulationAdapter } from '../src/adapters/simulationAdapter';
import { logger } from '../src/utils/logger';
import { Keypair, Connection, LAMPORTS_PER_SOL, Transaction, SystemProgram } from '@solana/web3.js';

class SolanaPayRobot {
  private solanaClient: SolanaClient;
  private robotController: RobotController;
  private humanoid: Humanoid;
  private serviceWallet: Keypair;
  private customerWallet: Keypair;
  private isRunning: boolean = false;

  // Service pricing (in lamports)
  private readonly PRICING = {
    MOVE_FORWARD: 1000000, // 0.001 SOL
    TURN: 500000,          // 0.0005 SOL
    GRIP_ACTION: 1500000,  // 0.0015 SOL
    COMPLEX_TASK: 5000000, // 0.005 SOL
    EMERGENCY_STOP: 0      // Free for safety
  };

  constructor() {
    this.initializePaymentSystem();
  }

  private initializePaymentSystem(): void {
    logger.info('Initializing Solana Pay Robot System...');

    // Generate wallets for service provider and customer
    this.serviceWallet = Keypair.generate();
    this.customerWallet = Keypair.generate();

    // Initialize Solana client for service provider
    this.solanaClient = new SolanaClient({
      network: 'devnet',
      commitment: 'confirmed',
      maxRetries: 3,
      retryDelay: 1000,
      wallet: this.serviceWallet
    });

    // Initialize robot system
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

    logger.info('Solana Pay system initialized', {
      serviceWallet: this.serviceWallet.publicKey.toString(),
      customerWallet: this.customerWallet.publicKey.toString()
    });
  }

  public async start(): Promise<void> {
    try {
      this.isRunning = true;
      logger.info('Starting Solana Pay robot service...');

      // Connect to blockchain and robot
      await this.connectSystems();
      
      // Fund wallets for demonstration
      await this.fundWallets();
      
      // Run payment demonstrations
      await this.demonstratePayPerAction();
      await this.demonstrateSubscriptionModel();
      await this.demonstrateMultiSigOperations();

      logger.info('Solana Pay demonstrations completed successfully');

    } catch (error) {
      logger.error('Solana Pay demonstration failed', error as Error);
      await this.handleError(error as Error);
    } finally {
      await this.cleanup();
    }
  }

  private async connectSystems(): Promise<void> {
    logger.info('Connecting Solana and robot systems...');

    await this.solanaClient.connect();
    await this.humanoid.connect();
    await this.robotController.addRobot('pay_humanoid', this.humanoid);

    const walletState = this.solanaClient.getWalletState();
    const robotState = this.humanoid.getState();

    logger.info('Systems connected successfully', {
      solanaConnected: walletState.connected,
      robotConnected: robotState.connected,
      network: walletState.network
    });
  }

  private async fundWallets(): Promise<void> {
    logger.info('Funding demonstration wallets...');

    const connection = new Connection('https://api.devnet.solana.com', 'confirmed');

    try {
      // Airdrop to service wallet
      const serviceAirdrop = await connection.requestAirdrop(
        this.serviceWallet.publicKey,
        2 * LAMPORTS_PER_SOL
      );
      await connection.confirmTransaction(serviceAirdrop);

      // Airdrop to customer wallet
      const customerAirdrop = await connection.requestAirdrop(
        this.customerWallet.publicKey,
        1 * LAMPORTS_PER_SOL
      );
      await connection.confirmTransaction(customerAirdrop);

      logger.info('Wallets funded successfully');

    } catch (error) {
      logger.warn('Airdrop failed, continuing with existing balances', { error });
    }

    // Log initial balances
    const serviceBalance = await this.solanaClient.getBalance();
    const customerBalance = await this.getCustomerBalance();

    logger.info('Initial wallet balances', {
      serviceBalance: serviceBalance / LAMPORTS_PER_SOL,
      customerBalance: customerBalance / LAMPORTS_PER_SOL
    });
  }

  private async demonstratePayPerAction(): Promise<void> {
    logger.info('Demonstrating Pay-Per-Action model...');

    const actions = [
      { type: 'MOVE_FORWARD', description: 'Move forward 1 meter', distance: 1.0 },
      { type: 'TURN', description: 'Turn 90 degrees right', angle: 90 },
      { type: 'GRIP_ACTION', description: 'Pick up object', force: 15.0 },
      { type: 'MOVE_FORWARD', description: 'Move forward 0.5 meters', distance: 0.5 }
    ];

    let totalCost = 0;
    let successfulActions = 0;

    for (const action of actions) {
      if (!this.isRunning) break;

      logger.info(`Processing action: ${action.description}`);

      // Step 1: Request payment from customer
      const paymentRequired = this.PRICING[action.type as keyof typeof this.PRICING];
      const paymentResult = await this.requestPayment(action.type, paymentRequired);

      if (!paymentResult.success) {
        logger.warn('Payment failed, skipping action', {
          action: action.type,
          reason: paymentResult.error
        });
        continue;
      }

      // Step 2: Verify payment on blockchain
      const verification = await this.verifyPayment(paymentResult.transactionHash);
      if (!verification.verified) {
        logger.error('Payment verification failed', {
          transactionHash: paymentResult.transactionHash,
          reason: verification.error
        });
        continue;
      }

      // Step 3: Execute robot action
      const actionResult = await this.executePaidAction(action);
      
      if (actionResult.success) {
        successfulActions++;
        totalCost += paymentRequired;
        
        logger.info('Paid action completed successfully', {
          action: action.type,
          cost: paymentRequired / LAMPORTS_PER_SOL,
          transaction: paymentResult.transactionHash
        });

        // Step 4: Log successful transaction
        await this.logServiceTransaction({
          action: action.type,
          cost: paymentRequired,
          transactionHash: paymentResult.transactionHash,
          customer: this.customerWallet.publicKey.toString(),
          timestamp: new Date()
        });

      } else {
        logger.error('Paid action failed, initiating refund', {
          action: action.type,
          error: actionResult.error
        });

        // Refund customer for failed action
        await this.processRefund(paymentResult.transactionHash, paymentRequired);
      }

      await this.delay(2000); // Pause between actions
    }

    logger.info('Pay-per-action demonstration completed', {
      successfulActions,
      totalActions: actions.length,
      totalCost: totalCost / LAMPORTS_PER_SOL,
      successRate: ((successfulActions / actions.length) * 100).toFixed(1) + '%'
    });
  }

  private async demonstrateSubscriptionModel(): Promise<void> {
    logger.info('Demonstrating Subscription Model...');

    // Simulate subscription purchase
    const subscriptionCost = 10 * LAMPORTS_PER_SOL; // 10 SOL for demo
    const subscriptionResult = await this.purchaseSubscription(subscriptionCost);

    if (!subscriptionResult.success) {
      logger.warn('Subscription purchase failed, skipping demonstration');
      return;
    }

    logger.info('Subscription purchased, executing subscription actions...');

    // Execute series of actions under subscription
    const subscriptionActions = [
      { type: 'MOVE_FORWARD', params: { distance: 1.5 } },
      { type: 'TURN', params: { angle: 45 } },
      { type: 'GRIP_ACTION', params: { force: 12.0 } },
      { type: 'COMPLEX_TASK', params: { task: 'wave_gesture' } }
    ];

    for (const action of subscriptionActions) {
      if (!this.isRunning) break;

      // Verify subscription is still active
      const subscriptionActive = await this.verifySubscription(subscriptionResult.subscriptionId);
      if (!subscriptionActive) {
        logger.warn('Subscription expired or invalid');
        break;
      }

      // Execute action without individual payment
      const result = await this.executePaidAction(action);
      
      if (result.success) {
        logger.info('Subscription action completed', {
          action: action.type,
          subscriptionId: subscriptionResult.subscriptionId
        });
      } else {
        logger.error('Subscription action failed', {
          action: action.type,
          error: result.error
        });
      }

      await this.delay(1500);
    }

    logger.info('Subscription model demonstration completed');
  }

  private async demonstrateMultiSigOperations(): Promise<void> {
    logger.info('Demonstrating Multi-Signature Operations...');

    // High-value operation requiring multiple approvals
    const highValueAction = {
      type: 'COMPLEX_TASK',
      description: 'High-precision assembly task',
      cost: 25 * LAMPORTS_PER_SOL, // 25 SOL
      requiredApprovals: 2,
      approvers: [
        this.serviceWallet.publicKey,
        this.customerWallet.publicKey,
        Keypair.generate().publicKey // Third party
      ]
    };

    logger.info('Initiating multi-signature operation', {
      action: highValueAction.description,
      cost: highValueAction.cost / LAMPORTS_PER_SOL,
      requiredApprovals: highValueAction.requiredApprovals
    });

    // Create multi-signature transaction
    const multiSigTx = await this.createMultiSigTransaction(highValueAction);
    
    // Collect signatures
    const signatures = await this.collectSignatures(multiSigTx, highValueAction.approvers);
    
    if (signatures.length >= highValueAction.requiredApprovals) {
      logger.info('Required signatures collected, executing high-value action');

      // Execute the high-value action
      const actionResult = await this.executeHighValueAction(highValueAction);
      
      if (actionResult.success) {
        logger.info('High-value action completed successfully', {
          action: highValueAction.type,
          signatures: signatures.length
        });

        // Complete the multi-signature transaction
        await this.completeMultiSigTransaction(multiSigTx, signatures);
      } else {
        logger.error('High-value action failed, canceling transaction');
        await this.cancelMultiSigTransaction(multiSigTx);
      }
    } else {
      logger.warn('Insufficient signatures for high-value action', {
        required: highValueAction.requiredApprovals,
        received: signatures.length
      });
    }

    logger.info('Multi-signature operations demonstration completed');
  }

  private async requestPayment(actionType: string, amount: number): Promise<any> {
    logger.info(`Requesting payment for ${actionType}`, {
      amount: amount / LAMPORTS_PER_SOL
    });

    // In a real implementation, this would create a payment request
    // and wait for the customer to sign the transaction
    // For demonstration, we'll simulate the payment

    const simulatedPayment = await this.simulateCustomerPayment(amount);
    
    if (simulatedPayment.success) {
      return {
        success: true,
        transactionHash: simulatedPayment.transactionHash,
        amount,
        actionType
      };
    } else {
      return {
        success: false,
        error: simulatedPayment.error,
        amount,
        actionType
      };
    }
  }

  private async verifyPayment(transactionHash: string): Promise<any> {
    logger.debug('Verifying payment transaction', { transactionHash });

    // Simulate blockchain verification
    await this.delay(1000); // Simulate network delay

    // In real implementation, this would check transaction status on-chain
    const verificationSuccess = Math.random() > 0.1; // 90% success rate

    if (verificationSuccess) {
      return {
        verified: true,
        transactionHash,
        confirmations: 32, // Simulated confirmation count
        timestamp: new Date()
      };
    } else {
      return {
        verified: false,
        transactionHash,
        error: 'Transaction verification failed',
        timestamp: new Date()
      };
    }
  }

  private async executePaidAction(action: any): Promise<any> {
    logger.info(`Executing paid action: ${action.type}`);

    try {
      // Convert action type to robot command
      const command = this.actionToRobotCommand(action);
      
      // Execute the command
      const result = await this.humanoid.executeCommand(command);
      
      return {
        success: true,
        result,
        action: action.type,
        timestamp: new Date()
      };

    } catch (error) {
      logger.error('Paid action execution failed', error as Error);
      return {
        success: false,
        error: (error as Error).message,
        action: action.type,
        timestamp: new Date()
      };
    }
  }

  private actionToRobotCommand(action: any): any {
    const commandMap: { [key: string]: any } = {
      'MOVE_FORWARD': {
        type: 'MOVEMENT',
        params: { direction: 'forward', distance: action.distance || 1.0, speed: 0.2 }
      },
      'TURN': {
        type: 'MOVEMENT',
        params: { rotation: action.angle || 90 }
      },
      'GRIP_ACTION': {
        type: 'MANIPULATION', 
        params: { action: 'grip', target: 'right_arm', force: action.force || 15.0 }
      },
      'COMPLEX_TASK': {
        type: 'SYSTEM',
        params: { task: action.task || 'default' }
      }
    };

    const commandTemplate = commandMap[action.type] || {
      type: 'SYSTEM',
      params: action.params
    };

    return {
      id: `paid_${action.type}_${Date.now()}`,
      ...commandTemplate,
      priority: 50,
      timeout: 10000,
      createdAt: new Date()
    };
  }

  private async simulateCustomerPayment(amount: number): Promise<any> {
    logger.debug('Simulating customer payment', { amount: amount / LAMPORTS_PER_SOL });

    // Check customer balance
    const customerBalance = await this.getCustomerBalance();
    
    if (customerBalance < amount) {
      return {
        success: false,
        error: 'Insufficient funds',
        required: amount,
        available: customerBalance
      };
    }

    // Simulate payment processing
    await this.delay(500);

    // In real implementation, this would create and send a transaction
    const transactionHash = `simulated_tx_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

    return {
      success: true,
      transactionHash,
      amount,
      timestamp: new Date()
    };
  }

  private async getCustomerBalance(): Promise<number> {
    // In real implementation, this would query the blockchain
    // For demonstration, return a simulated balance
    return 0.5 * LAMPORTS_PER_SOL; // 0.5 SOL
  }

  private async logServiceTransaction(transactionData: any): Promise<void> {
    logger.info('Logging service transaction', {
      action: transactionData.action,
      cost: transactionData.cost / LAMPORTS_PER_SOL,
      customer: transactionData.customer
    });

    // In production, this would store transaction data in a database
    // and potentially log it to the blockchain for immutability

    await this.delay(100); // Simulate logging delay
  }

  private async processRefund(transactionHash: string, amount: number): Promise<void> {
    logger.info('Processing refund', {
      transactionHash,
      amount: amount / LAMPORTS_PER_SOL
    });

    // In real implementation, this would create a refund transaction
    // For demonstration, we'll just log the refund

    await this.delay(500); // Simulate refund processing

    logger.info('Refund processed successfully', {
      transactionHash,
      amount: amount / LAMPORTS_PER_SOL
    });
  }

  private async purchaseSubscription(cost: number): Promise<any> {
    logger.info('Processing subscription purchase', { cost: cost / LAMPORTS_PER_SOL });

    // Simulate subscription purchase
    const paymentResult = await this.simulateCustomerPayment(cost);
    
    if (paymentResult.success) {
      const subscriptionId = `sub_${Date.now()}_${Math.random().toString(36).substr(2, 6)}`;
      
      return {
        success: true,
        subscriptionId,
        cost,
        expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000) // 24 hours
      };
    } else {
      return {
        success: false,
        error: paymentResult.error
      };
    }
  }

  private async verifySubscription(subscriptionId: string): Promise<boolean> {
    // Simulate subscription verification
    await this.delay(200);
    
    // In real implementation, this would check subscription status in database/blockchain
    return true; // Always valid for demonstration
  }

  private async createMultiSigTransaction(action: any): Promise<any> {
    logger.info('Creating multi-signature transaction', { action: action.type });

    // Simulate multi-signature transaction creation
    const transactionId = `multisig_tx_${Date.now()}`;
    
    return {
      id: transactionId,
      action: action.type,
      cost: action.cost,
      requiredApprovals: action.requiredApprovals,
      approvers: action.approvers,
      createdAt: new Date(),
      status: 'pending'
    };
  }

  private async collectSignatures(transaction: any, approvers: any[]): Promise<string[]> {
    logger.info('Collecting signatures for multi-signature transaction');

    const signatures: string[] = [];
    
    // Simulate signature collection from approvers
    for (const approver of approvers.slice(0, 2)) { // First two approvers for demo
      const signature = await this.simulateApproval(approver, transaction);
      if (signature) {
        signatures.push(signature);
        logger.info('Signature collected', { approver: approver.toString() });
      }
      
      if (signatures.length >= transaction.requiredApprovals) {
        break;
      }
    }

    return signatures;
  }

  private async simulateApproval(approver: any, transaction: any): Promise<string> {
    // Simulate approval process with 80% success rate
    await this.delay(300);
    
    if (Math.random() > 0.2) {
      return `sig_${approver.toString().substr(0, 8)}_${Date.now()}`;
    }
    
    return '';
  }

  private async executeHighValueAction(action: any): Promise<any> {
    logger.info('Executing high-value action', { action: action.type });

    try {
      // Simulate complex, high-value operation
      await this.delay(2000);
      
      // 95% success rate for high-value actions
      const success = Math.random() > 0.05;
      
      if (success) {
        return {
          success: true,
          result: 'high_value_action_completed',
          quality: 0.95, // Simulated quality metric
          duration: 2000
        };
      } else {
        throw new Error('High-value action failed due to precision requirements');
      }

    } catch (error) {
      logger.error('High-value action execution failed', error as Error);
      return {
        success: false,
        error: (error as Error).message
      };
    }
  }

  private async completeMultiSigTransaction(transaction: any, signatures: string[]): Promise<void> {
    logger.info('Completing multi-signature transaction', {
      transactionId: transaction.id,
      signatureCount: signatures.length
    });

    // Simulate transaction completion
    await this.delay(500);
    
    logger.info('Multi-signature transaction completed successfully', {
      transactionId: transaction.id,
      action: transaction.action,
      cost: transaction.cost / LAMPORTS_PER_SOL
    });
  }

  private async cancelMultiSigTransaction(transaction: any): Promise<void> {
    logger.info('Canceling multi-signature transaction', { transactionId: transaction.id });

    // Simulate transaction cancellation
    await this.delay(300);
    
    logger.info('Multi-signature transaction canceled', { transactionId: transaction.id });
  }

  private async handleError(error: Error): Promise<void> {
    logger.error('Solana Pay system error', error);
    
    try {
      // Safe the robot
      await this.humanoid.emergencyStop();
      await this.humanoid.stand();
      
      // Cancel any pending transactions
      await this.cancelPendingTransactions();
      
      logger.info('Error handling completed');
    } catch (safeError) {
      logger.error('Error during error handling', safeError as Error);
    }
  }

  private async cancelPendingTransactions(): Promise<void> {
    logger.info('Canceling any pending transactions...');
    // In real implementation, this would cancel pending blockchain transactions
    await this.delay(200);
  }

  private async cleanup(): Promise<void> {
    this.isRunning = false;
    
    logger.info('Cleaning up Solana Pay system...');
    
    try {
      await this.humanoid.stand();
      await this.humanoid.disconnect();
      await this.robotController.disconnect();
      await this.solanaClient.disconnect();
      
      logger.info('Solana Pay system cleanup completed');
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
  const solanaPayRobot = new SolanaPayRobot();
  
  solanaPayRobot.start().catch(error => {
    logger.error('Fatal error in Solana Pay demonstration', error);
    process.exit(1);
  });

  // Handle graceful shutdown
  process.on('SIGINT', async () => {
    logger.info('Received shutdown signal for Solana Pay...');
    await solanaPayRobot.cleanup();
    process.exit(0);
  });
}

export { SolanaPayRobot };