import { PublicKey } from "@solana/web3.js";

export const sleep = (ms: number): Promise<void> => {
  return new Promise(resolve => setTimeout(resolve, ms));
};

export const generateMockHash = (prefix: string = 'SimTX'): string => {
  const chars = '0123456789abcdef';
  let hash = '';
  for (let i = 0; i < 16; i++) {
    hash += chars[Math.floor(Math.random() * chars.length)];
  }
  return `${prefix}_${hash}`;
};

export const formatLamports = (lamports: number): string => {
  const synrAmount = lamports / 1e9;
  return `$${synrAmount.toFixed(2)} SYNR`;
};

export const retry = async <T>(
  fn: () => Promise<T>,
  retries: number = 3,
  delay: number = 1000
): Promise<T> => {
  try {
    return await fn();
  } catch (error) {
    if (retries <= 0) {
      throw error;
    }
    await sleep(delay);
    return retry(fn, retries - 1, delay * 2);
  }
};

export const validateSolanaAddress = (address: string): boolean => {
  try {
    new PublicKey(address);
    return true;
  } catch {
    return false;
  }
};

export const calculateNetworkFee = (
  amount: number, 
  priority: 'low' | 'medium' | 'high' = 'medium'
): number => {
  const baseFees = {
    low: 5000,
    medium: 10000,
    high: 20000
  };
  return baseFees[priority] + Math.floor(amount * 100);
};

export class Logger {
  private debugEnabled: boolean;
  
  constructor(debug: boolean = false) {
    this.debugEnabled = debug || process.env.DEBUG === 'true';
  }

  static info(...args: any[]): void {
    console.log('ℹ️', ...args);
  }

  static success(...args: any[]): void {
    console.log('✅', ...args);
  }

  static error(...args: any[]): void {
    console.error('❌', ...args);
  }

  static warn(...args: any[]): void {
    console.warn('⚠️', ...args);
  }

  debug(...args: any[]): void {
    if (this.debugEnabled) {
      console.debug('🐞', ...args);
    }
  }

  setDebug(enabled: boolean): void {
    this.debugEnabled = enabled;
  }
}

export class MockTransactionBuilder {
  private transactionCount: number = 0;

  buildMockTransaction(params: {
    from: string;
    to: string;
    amount: number;
    type: 'transfer' | 'payment' | 'staking';
  }): any {
    this.transactionCount++;
    
    return {
      signature: generateMockHash('MOCK'),
      type: params.type,
      from: params.from,
      to: params.to,
      amount: params.amount,
      fee: calculateNetworkFee(params.amount),
      timestamp: Date.now(),
      nonce: this.transactionCount,
      status: 'confirmed',
      block: Math.floor(1000000 + Math.random() * 1000000)
    };
  }

  getTransactionCount(): number {
    return this.transactionCount;
  }

  reset(): void {
    this.transactionCount = 0;
  }
}

export const CONSTANTS = {
  DEFAULT_RPC: "https://api.mainnet-beta.solana.com",
  SYNR_DECIMALS: 9,
  SYNR_SYMBOL: "$SYNR",
  MAX_RETRIES: 5,
  DEFAULT_TIMEOUT: 30000,
  NETWORK_FEES: {
    MAINNET: 5000,
    DEVNET: 1000,
    TESTNET: 1000
  }
} as const;

export const safeAsync = async <T>(fn: () => Promise<T>): Promise<T | null> => {
  try {
    return await fn();
  } catch (error) {
    Logger.error('Safe async error:', error);
    return null;
  }
};

export const generatePaymentMemo = (robotId: string, service: string): string => {
  const timestamp = new Date().toISOString().split('T')[0];
  return `SYNR-${service}-${robotId}-${timestamp}`;
};

export const parsePaymentMemo = (memo: string): { service: string; robotId: string; date: string } | null => {
  const parts = memo.split('-');
  if (parts.length >= 4 && parts[0] === 'SYNR') {
    return {
      service: parts[1],
      robotId: parts[2],
      date: parts[3]
    };
  }
  return null;
};