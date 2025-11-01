import { SolanaClient } from '../src/core/solanaClient';
import { Keypair, Connection, LAMPORTS_PER_SOL } from '@solana/web3.js';
import { logger } from '../src/utils/logger';
import { SOLANA_NETWORKS, DEFAULT_SOLANA_NETWORK } from '../src/constants';

describe('Solana Integration Tests', () => {
  let solanaClient: SolanaClient;
  let testWallet: Keypair;
  let connection: Connection;

  beforeAll(() => {
    testWallet = Keypair.generate();
    connection = new Connection(SOLANA_NETWORKS.devnet, 'confirmed');
  });

  beforeEach(() => {
    solanaClient = new SolanaClient({
      network: SOLANA_NETWORKS.devnet,
      commitment: 'confirmed',
      maxRetries: 3,
      retryDelay: 1000,
      wallet: testWallet
    });
  });

  afterEach(async () => {
    await solanaClient.disconnect();
  });

  describe('Wallet Management', () => {
    test('should connect to Solana network', async () => {
      await solanaClient.connect();
      const walletState = solanaClient.getWalletState();
      
      expect(walletState.connected).toBe(true);
      expect(walletState.publicKey).toEqual(testWallet.publicKey);
      expect(walletState.network).toBe(SOLANA_NETWORKS.devnet);
    });

    test('should get wallet balance', async () => {
      await solanaClient.connect();
      
      // Airdrop some SOL for testing (on devnet)
      try {
        const signature = await connection.requestAirdrop(
          testWallet.publicKey,
          1 * LAMPORTS_PER_SOL
        );
        await connection.confirmTransaction(signature);
      } catch (error) {
        // Airdrop might fail on devnet, that's OK for test
        console.log('Airdrop failed, continuing with test...');
      }

      const balance = await solanaClient.getBalance();
      expect(typeof balance).toBe('number');
      expect(balance).toBeGreaterThanOrEqual(0);
    });

    test('should handle wallet disconnection', async () => {
      await solanaClient.connect();
      expect(solanaClient.getWalletState().connected).toBe(true);
      
      await solanaClient.disconnect();
      expect(solanaClient.getWalletState().connected).toBe(false);
    });
  });

  describe('Transaction Processing', () => {
    test('should create and sign transactions', async () => {
      await solanaClient.connect();
      
      const recipient = Keypair.generate().publicKey;
      const transactionData = {
        recipient,
        amount: 0.001 * LAMPORTS_PER_SOL,
        description: 'Test payment',
        metadata: { test: true }
      };

      const transaction = await solanaClient.createPaymentTransaction(transactionData);
      
      expect(transaction.transaction).toBeDefined();
      expect(transaction.signers).toContain(testWallet);
      expect(transaction.description).toBe('Test payment');
      expect(transaction.metadata.test).toBe(true);
    });

    test('should sign messages with wallet', async () => {
      await solanaClient.connect();
      
      const message = 'Test message for signing';
      const signature = await solanaClient.signMessage(message);
      
      expect(signature).toBeDefined();
      expect(typeof signature).toBe('string');
      expect(signature.length).toBeGreaterThan(0);
    });

    test('should handle transaction failures gracefully', async () => {
      await solanaClient.connect();
      
      // Try to send transaction with insufficient funds
      const recipient = Keypair.generate().publicKey;
      const transactionData = {
        recipient,
        amount: 1000 * LAMPORTS_PER_SOL, // Way more than we have
        description: 'Should fail'
      };

      await expect(solanaClient.sendTransaction(transactionData))
        .rejects
        .toMatchObject({
          code: 'INSUFFICIENT_FUNDS'
        });
    });
  });

  describe('Robot Action Transactions', () => {
    test('should create robot action transaction', async () => {
      await solanaClient.connect();
      
      const robotAction = {
        robotId: 'humanoid_001',
        action: 'move_forward',
        parameters: { distance: 1.5, speed: 0.3 },
        verification: { sensor_data: 'clean', safety_checks: 'passed' }
      };

      const transaction = await solanaClient.createRobotActionTransaction(robotAction);
      
      expect(transaction.transaction).toBeDefined();
      expect(transaction.metadata.robotId).toBe('humanoid_001');
      expect(transaction.metadata.action).toBe('move_forward');
    });

    test('should verify robot action on-chain', async () => {
      await solanaClient.connect();
      
      const actionHash = 'test_action_hash_123';
      const verification = await solanaClient.verifyRobotAction(actionHash);
      
      // In simulation, this should return a mock verification result
      expect(veraction).toBeDefined();
      expect(verification.verified).toBe(true);
      expect(verification.timestamp).toBeInstanceOf(Date);
    });

    test('should handle multiple concurrent transactions', async () => {
      await solanaClient.connect();
      
      const actions = [
        { robotId: 'robot_1', action: 'grip', parameters: { force: 20 } },
        { robotId: 'robot_2', action: 'move', parameters: { direction: 'left' } },
        { robotId: 'robot_3', action: 'sense', parameters: { sensor: 'camera' } }
      ];

      const transactionPromises = actions.map(action => 
        solanaClient.createRobotActionTransaction(action)
      );

      const transactions = await Promise.all(transactionPromises);
      
      expect(transactions).toHaveLength(3);
      transactions.forEach(tx => {
        expect(tx.transaction).toBeDefined();
        expect(tx.metadata.robotId).toBeDefined();
      });
    });
  });

  describe('Event System', () => {
    test('should emit transaction events', (done) => {
      solanaClient.on('transaction_confirmed', (event) => {
        expect(event.signature).toBeDefined();
        expect(event.slot).toBeGreaterThan(0);
        done();
      });

      // Simulate a transaction confirmation
      setTimeout(() => {
        solanaClient.emit('transaction_confirmed', {
          signature: 'test_signature_123',
          slot: 123456,
          confirmationStatus: 'confirmed'
        });
      }, 100);
    });

    test('should handle network change events', (done) => {
      solanaClient.on('network_changed', (event) => {
        expect(event.network).toBe(SOLANA_NETWORKS.mainnet);
        done();
      });

      solanaClient.updateNetwork(SOLANA_NETWORKS.mainnet);
    });
  });

  describe('Error Handling and Retries', () => {
    test('should retry failed transactions', async () => {
      const failingClient = new SolanaClient({
        network: 'http://invalid-endpoint:8899', // Invalid endpoint
        commitment: 'confirmed',
        maxRetries: 2,
        retryDelay: 100
      });

      await expect(failingClient.connect())
        .rejects
        .toMatchObject({
          code: 'SOLANA_CONNECTION_FAILED'
        });
    });

    test('should handle network timeouts', async () => {
      const slowClient = new SolanaClient({
        network: SOLANA_NETWORKS.devnet,
        commitment: 'confirmed',
        maxRetries: 1,
        retryDelay: 100,
        timeout: 1 // 1ms timeout to force failure
      });

      await expect(slowClient.getBalance())
        .rejects
        .toMatchObject({
          code: 'TIMEOUT'
        });
    });
  });

  describe('Integration with Robot System', () => {
    test('should log robot actions to blockchain', async () => {
      await solanaClient.connect();
      
      const robotAction = {
        robotId: 'test_bot_001',
        action: 'assembly_complete',
        parameters: {
          part: 'engine_block',
          quality: 0.95,
          duration: 125.7
        },
        verification: {
          quality_check: 'passed',
          safety_inspection: 'passed'
        }
      };

      const result = await solanaClient.logRobotAction(robotAction);
      
      expect(result.success).toBe(true);
      expect(result.transactionHash).toBeDefined();
      expect(result.actionId).toBe(robotAction.robotId);
    });

    test('should verify action completion on-chain', async () => {
      await solanaClient.connect();
      
      const actionId = 'test_action_789';
      const verification = await solanaClient.verifyActionCompletion(actionId);
      
      expect(verification.completed).toBe(true);
      expect(verification.verifiedBy).toContain('solana');
      expect(verification.timestamp).toBeInstanceOf(Date);
    });
  });
});