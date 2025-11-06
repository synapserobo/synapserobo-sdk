import { TokenBridge } from "../src/core/tokenBridge";
import { 
  sleep, 
  generateMockHash, 
  formatLamports, 
  retry, 
  Logger,
  validateSolanaAddress,
  MockTransactionBuilder 
} from "../src/core/tokenBridge.utils";

const TEST_CONFIG = {
  rpcUrl: "https://api.testnet.solana.com", 
  simulate: true,
  networkName: "testnet" as const
};

describe('SYNR TokenBridge', () => {
  let bridge: TokenBridge;
  let logger: Logger;

  beforeEach(() => {
    logger = new Logger(false);
    bridge = new TokenBridge({ ...TEST_CONFIG, logger });
  });

  afterEach(async () => {
    await bridge.disconnect();
  });

  describe('Initialization', () => {
    test('should initialize with simulation mode', () => {
      expect(bridge.isSimulated()).toBe(true);
    });

    test('should return correct version', () => {
      expect(bridge.getVersion()).toMatch(/\d+\.\d+\.\d+-SYNR/);
    });
  });

  describe('Payment Processing', () => {
    test('should process simulated payment successfully', async () => {
      const result = await bridge.payForSync({
        robotId: "testBot01",
        payer: "testWallet123",
        amount: 10.5,
        memo: "Test payment"
      });

      expect(result.status).toBe("success");
      expect(result.simulated).toBe(true);
      expect(result.txHash).toBeDefined();
      expect(result.amount).toBe(10.5);
    });

    test('should reject zero amount payments', async () => {
      await expect(
        bridge.payForSync({
          robotId: "testBot01",
          payer: "testWallet123", 
          amount: 0,
          memo: "Invalid payment"
        })
      ).rejects.toThrow('Payment amount must be positive');
    });
  });

  describe('Balance Checks', () => {
    test('should return simulated balance', async () => {
      const balance = await bridge.checkBalance("testAddress123");
      expect(balance.balance).toBeGreaterThan(0);
      expect(balance.unit).toBe("SYNR");
      expect(balance.simulated).toBe(true);
    });
  });

  describe('Transaction Verification', () => {
    test('should verify simulated transaction', async () => {
      const verification = await bridge.verifyTransaction("test_tx_hash_123");
      expect(verification.confirmed).toBe(true);
      expect(verification.simulated).toBe(true);
      expect(verification.txHash).toBe("test_tx_hash_123");
    });
  });

  describe('Utility Functions', () => {
    test('sleep function should delay execution', async () => {
      const start = Date.now();
      await sleep(100);
      const end = Date.now();
      expect(end - start).toBeGreaterThanOrEqual(100);
    });

    test('generateMockHash should create valid format', () => {
      const hash = generateMockHash('TEST');
      expect(hash).toMatch(/^TEST_[0-9a-f]{16}$/);
    });

    test('formatLamports should format correctly', () => {
      const formatted = formatLamports(1e9);
      expect(formatted).toBe('$1.00 SYNR');
    });

    test('retry should work with successful function', async () => {
      let attempts = 0;
      const successFn = async () => {
        attempts++;
        return 'success';
      };

      const result = await retry(successFn);
      expect(result).toBe('success');
      expect(attempts).toBe(1);
    });
  });

  describe('Event System', () => {
    test('should emit payment events', (done) => {
      bridge.on('paid', (data) => {
        expect(data.status).toBe('success');
        done();
      });

      bridge.payForSync({
        robotId: "eventBot01",
        payer: "eventWallet",
        amount: 1.0
      });
    });
  });

  describe('Health Checks', () => {
    test('should return healthy status', async () => {
      const health = await bridge.healthCheck();
      expect(health.healthy).toBe(true);
    });
  });
});

describe('TokenBridge Utilities', () => {
  test('MockTransactionBuilder should create transactions', () => {
    const builder = new MockTransactionBuilder();
    const tx = builder.buildMockTransaction({
      from: 'addr1',
      to: 'addr2', 
      amount: 100,
      type: 'transfer'
    });

    expect(tx.signature).toBeDefined();
    expect(tx.amount).toBe(100);
    expect(builder.getTransactionCount()).toBe(1);
  });

  test('Logger should not throw', () => {
    expect(() => {
      Logger.info('Test info');
      Logger.success('Test success');
      Logger.error('Test error');
      Logger.warn('Test warning');
    }).not.toThrow();
  });
});