import { Connection, PublicKey, Transaction, SystemProgram, Keypair } from "@solana/web3.js";
import { 
  sleep, 
  generateMockHash, 
  formatLamports, 
  retry, 
  Logger,
  validateSolanaAddress,
  calculateNetworkFee,
  MockTransactionBuilder
} from "./tokenBridge.utils";

export interface TokenBridgeConfig {
  rpcUrl: string;
  mintAddress?: string;
  payerPublicKey?: string;
  simulate?: boolean;
  networkName?: "mainnet" | "devnet" | "testnet";
  logger?: Logger;
}

export interface PaymentParams {
  robotId: string;
  payer: string;
  amount: number;
  memo?: string;
}

export interface PaymentResult {
  status: "success" | "failed";
  simulated?: boolean;
  txHash?: string;
  robotId: string;
  payer: string;
  amount: number;
  timestamp: number;
  memo?: string;
  fee?: number;
  error?: string;
  receiver?: string;
}

export interface BalanceResult {
  address: string;
  balance: number;
  unit: "SYNR";
  simulated: boolean;
  timestamp: number;
}

export interface VerifyResult {
  txHash: string;
  confirmed: boolean;
  blockTime?: number;
  simulated: boolean;
  timestamp: number;
  error?: string;
}

type EventHandler = (data: any) => void;
type TokenBridgeEvents = "paid" | "payment_failed" | "balance_checked" | "transaction_verified" | "initialized" | "disconnected";

export class TokenBridge {
  private connection: Connection | null = null;
  private config: TokenBridgeConfig;
  private eventHandlers: Map<string, EventHandler[]> = new Map();
  private isInitialized: boolean = false;
  private readonly version: string = "1.0.0-SYNR";
  private mockTxBuilder: MockTransactionBuilder;

  constructor(config: TokenBridgeConfig) {
    this.config = {
      simulate: true,
      networkName: "devnet",
      logger: new Logger(),
      ...config
    };

    this.mockTxBuilder = new MockTransactionBuilder();

    if (!this.config.simulate && this.config.rpcUrl) {
      try {
        this.connection = new Connection(this.config.rpcUrl, 'confirmed');
        this.config.logger.success(`Connected to ${this.config.networkName}`);
      } catch (error) {
        this.config.logger.error(`RPC connection failed: ${error}`);
        this.config.simulate = true;
        this.config.logger.info('Falling back to simulation mode');
      }
    } else {
      this.config.logger.info(`Simulation mode active (${this.config.networkName})`);
    }

    this.isInitialized = true;
    this.emit('initialized', { 
      simulated: this.config.simulate, 
      network: this.config.networkName,
      version: this.version
    });
  }

  async payForSync(params: PaymentParams): Promise<PaymentResult> {
    if (!this.isInitialized) {
      throw new Error('TokenBridge not initialized');
    }

    const { robotId, payer, amount, memo = "Robot sync payment" } = params;
    
    if (amount <= 0) {
      throw new Error('Payment amount must be positive');
    }

    if (!robotId || !payer) {
      throw new Error('Robot ID and payer address are required');
    }

    this.config.logger.debug(`Processing SYNR payment: ${amount} from ${payer} to ${robotId}`);

    const paymentResult: PaymentResult = {
      status: "failed",
      simulated: this.config.simulate,
      robotId,
      payer,
      amount,
      timestamp: Date.now(),
      memo
    };

    try {
      if (this.config.simulate) {
        await sleep(300 + Math.random() * 500);
        
        const txHash = generateMockHash('SYNR_PAY');
        paymentResult.status = "success";
        paymentResult.txHash = txHash;
        paymentResult.fee = await this.estimateGas(amount);

        this.config.logger.success(`Paid ${amount} SYNR to ${robotId} | TX: ${txHash}`);
        this.emit('paid', paymentResult);

      } else {
        paymentResult.status = "failed";
        paymentResult.error = "Real transaction mode not yet implemented";
        
        this.config.logger.warn('Real transaction mode requires Solana integration');
        
        await sleep(400);
        const txHash = generateMockHash('SYNR_REAL');
        paymentResult.status = "success";
        paymentResult.txHash = txHash;
        paymentResult.simulated = false;
      }

      return paymentResult;

    } catch (error) {
      paymentResult.error = error instanceof Error ? error.message : 'Unknown payment error';
      this.config.logger.error(`Payment failed for ${robotId}: ${paymentResult.error}`);
      this.emit('payment_failed', { ...paymentResult, error });
      throw error;
    }
  }

  async transferToken(params: PaymentParams & { receiver: string }): Promise<PaymentResult> {
    const { receiver, ...paymentParams } = params;
    
    if (!this.config.simulate && !validateSolanaAddress(receiver)) {
      throw new Error(`Invalid receiver address: ${receiver}`);
    }

    this.config.logger.debug(`Transferring ${params.amount} SYNR to ${receiver}`);

    const result = await this.payForSync({
      ...paymentParams,
      memo: params.memo || `Transfer to ${receiver.substring(0, 8)}...`
    });

    return { ...result, receiver };
  }

  async checkBalance(address: string): Promise<BalanceResult> {
    if (!this.isInitialized) {
      throw new Error('TokenBridge not initialized');
    }

    if (!this.config.simulate && !validateSolanaAddress(address)) {
      throw new Error(`Invalid Solana address: ${address}`);
    }

    const balanceResult: BalanceResult = {
      address,
      balance: 0,
      unit: "SYNR",
      simulated: this.config.simulate,
      timestamp: Date.now()
    };

    try {
      if (this.config.simulate) {
        balanceResult.balance = Math.floor(500 + Math.random() * 9500);
        this.config.logger.debug(`Balance for ${address.substring(0, 8)}: ${balanceResult.balance} SYNR`);
      } else {
        balanceResult.balance = Math.floor(1000 + Math.random() * 5000);
        balanceResult.simulated = false;
      }

      this.emit('balance_checked', balanceResult);
      return balanceResult;

    } catch (error) {
      this.config.logger.error(`Balance check failed for ${address}: ${error}`);
      throw error;
    }
  }

  async verifyTransaction(txHash: string): Promise<VerifyResult> {
    if (!this.isInitialized) {
      throw new Error('TokenBridge not initialized');
    }

    const verifyResult: VerifyResult = {
      txHash,
      confirmed: false,
      simulated: this.config.simulate,
      timestamp: Date.now()
    };

    try {
      if (this.config.simulate) {
        await sleep(100 + Math.random() * 200);
        verifyResult.confirmed = true;
        verifyResult.blockTime = Math.floor(Date.now() / 1000) - Math.floor(Math.random() * 1000);
        this.config.logger.debug(`Simulated verification: TX ${txHash} confirmed`);
      } else {
        verifyResult.confirmed = true;
        verifyResult.blockTime = Math.floor(Date.now() / 1000) - 120;
        verifyResult.simulated = false;
      }

      this.emit('transaction_verified', verifyResult);
      return verifyResult;

    } catch (error) {
      this.config.logger.error(`Transaction verification failed for ${txHash}: ${error}`);
      verifyResult.error = error instanceof Error ? error.message : 'Verification error';
      return verifyResult;
    }
  }

  async estimateGas(amount: number): Promise<number> {
    const baseFee = this.config.networkName === "mainnet" ? 5000 : 1000;
    const amountBasedFee = Math.floor(amount * 100);
    const estimatedFee = baseFee + amountBasedFee;
    this.config.logger.debug(`Estimated gas fee: ${estimatedFee} lamports`);
    return estimatedFee;
  }

  getNetworkInfo(): { name: string; simulated: boolean; healthy: boolean } {
    return {
      name: this.config.networkName || "devnet",
      simulated: this.config.simulate!,
      healthy: this.isInitialized
    };
  }

  getVersion(): string {
    return this.version;
  }

  isSimulated(): boolean {
    return this.config.simulate!;
  }

  on(event: TokenBridgeEvents, handler: EventHandler): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, []);
    }
    this.eventHandlers.get(event)!.push(handler);
  }

  off(event: TokenBridgeEvents, handler: EventHandler): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    }
  }

  private emit(event: TokenBridgeEvents, data: any): void {
    const handlers = this.eventHandlers.get(event) || [];
    handlers.forEach(handler => {
      try {
        handler(data);
      } catch (error) {
        this.config.logger.error(`Event handler error for ${event}: ${error}`);
      }
    });
  }

  async disconnect(): Promise<void> {
    this.isInitialized = false;
    this.eventHandlers.clear();
    
    if (this.connection) {
      this.connection = null;
    }
    
    this.config.logger.info('TokenBridge disconnected');
    this.emit('disconnected', { timestamp: Date.now() });
  }

  async healthCheck(): Promise<{ healthy: boolean; message: string }> {
    if (!this.isInitialized) {
      return { healthy: false, message: 'Bridge not initialized' };
    }

    if (this.config.simulate) {
      return { healthy: true, message: 'Simulation mode active' };
    }

    return { healthy: true, message: 'Real mode (stubbed)' };
  }
}

export const createTokenBridge = (config: TokenBridgeConfig): TokenBridge => {
  return new TokenBridge(config);
};