/**
 * @file solanaClient.ts
 * @description Core Solana client for SynapseRobo SDK
 * Handles wallet connection, transaction building, message signing, 
 * blockchain logging, micro-payments, retry queues, and simulation mode.
 */

import { 
  Connection, 
  PublicKey, 
  Transaction, 
  SystemProgram, 
  LAMPORTS_PER_SOL,
  TransactionInstruction,
  TransactionSignature,
  ConfirmedSignatureInfo,
  SignatureResult,
  Keypair
} from '@solana/web3.js';
import { Token, TOKEN_PROGRAM_ID } from '@solana/spl-token';
import { Subject, Observable, from, of, throwError, timer, interval, BehaviorSubject } from 'rxjs';
import { map, catchError, retry, timeout, switchMap, takeUntil, filter, tap, mergeMap, finalize } from 'rxjs/operators';
import { v4 as uuidv4 } from 'uuid';
import { logger } from '@utils/logger';
import { 
  SolanaConfig, 
  TransactionResult, 
  BlockchainLog, 
  PaymentRequest, 
  PaymentResult,
  WalletStatus,
  NetworkStatus
} from '@utils/types';

interface TransactionQueueItem {
  transaction: Transaction;
  description: string;
  retries: number;
  timestamp: number;
  id: string;
}

interface PendingSignature {
  signature: string;
  resolve: (result: TransactionResult) => void;
  reject: (error: Error) => void;
  timeout: any;
}

export class SolanaClient {
  private config: SolanaConfig;
  private connection: Connection | null = null;
  private wallet: Keypair | null = null;
  private transactionQueue: TransactionQueueItem[] = [];
  private processingQueue = false;
  private pendingSignatures = new Map<string, PendingSignature>();
  private eventSubject = new Subject<any>();
  private statusSubject = new BehaviorSubject<WalletStatus>('disconnected');
  private networkStatus: NetworkStatus = { healthy: false, latency: 0, lastCheck: 0 };

  /**
   * Creates a new Solana client instance
   * @param config - Solana configuration
   */
  constructor(config: Partial<SolanaConfig> = {}) {
    this.config = {
      network: config.network || 'devnet',
      commitment: config.commitment || 'confirmed',
      endpoint: config.endpoint || this.getDefaultEndpoint(config.network),
      maxRetries: config.maxRetries ?? 3,
      retryDelay: config.retryDelay ?? 1000,
      confirmTimeout: config.confirmTimeout ?? 30000,
      simulationMode: config.simulationMode ?? false,
      microPaymentEnabled: config.microPaymentEnabled ?? true,
      paymentRecipient: config.paymentRecipient || null,
      paymentAmount: config.paymentAmount ?? 0.001, // SOL
    };

    this.startNetworkMonitor();
    this.startTransactionProcessor();
  }

  /**
   * Connects to Solana network with wallet
   * @param wallet - Optional wallet keypair
   * @returns Connection status
   */
  public async connect(wallet?: Keypair): Promise<WalletStatus> {
    logger.info('Connecting to Solana network...', { network: this.config.network });

    try {
      // Create connection
      this.connection = new Connection(this.config.endpoint!, {
        commitment: this.config.commitment,
        confirmTransactionInitialTimeout: this.config.confirmTimeout,
      });

      // Test connection
      const version = await this.connection.getVersion();
      logger.debug('Solana node version', { version });

      // Set or generate wallet
      if (wallet) {
        this.wallet = wallet;
      } else if (!this.wallet) {
        this.wallet = this.config.simulationMode ? Keypair.generate() : null;
      }

      const publicKey = this.wallet ? this.wallet.publicKey.toBase58() : null;
      const balance = this.wallet ? await this.getBalance(this.wallet.publicKey) : 0;

      this.statusSubject.next('connected');
      this.networkStatus.healthy = true;
      this.networkStatus.lastCheck = Date.now();

      logger.info('Solana client connected successfully', {
        publicKey,
        balance: balance / LAMPORTS_PER_SOL,
        network: this.config.network,
        simulation: this.config.simulationMode,
      });

      return 'connected';
    } catch (error) {
      logger.error('Failed to connect to Solana', error as Error);
      this.statusSubject.next('error');
      throw error;
    }
  }

  /**
   * Disconnects from Solana network
   */
  public disconnect(): void {
    this.connection = null;
    this.wallet = null;
    this.transactionQueue = [];
    this.pendingSignatures.clear();
    this.statusSubject.next('disconnected');
    
    logger.info('Solana client disconnected');
  }

  /**
   * Signs a message with wallet
   * @param message - Message to sign
   * @returns Signature in base58
   */
  public signMessage(message: string | Uint8Array): string {
    this.validateConnection();
    
    if (!this.wallet) {
      throw new Error('Wallet not connected');
    }

    const messageBytes = typeof message === 'string' ? new TextEncoder().encode(message) : message;
    const signature = this.wallet.sign(messageBytes);
    
    logger.debug('Message signed', { 
      message: typeof message === 'string' ? message : message.length + ' bytes',
      signature: Buffer.from(signature).toString('base64')
    });

    return Buffer.from(signature).toString('base64');
  }

  /**
   * Builds and sends a transaction
   * @param instructions - Transaction instructions
   * @param description - Transaction description
   * @returns Transaction result
   */
  public async sendTransaction(
    instructions: TransactionInstruction[],
    description: string = 'Custom transaction'
  ): Promise<TransactionResult> {
    this.validateConnection();
    
    if (!this.wallet) {
      throw new Error('Wallet not connected');
    }

    const transaction = new Transaction();
    transaction.recentBlockhash = (await this.connection!.getLatestBlockhash()).blockhash;
    transaction.feePayer = this.wallet.publicKey;

    instructions.forEach(ix => transaction.add(ix));

    if (this.config.simulationMode) {
      return this.simulateTransaction(transaction, description);
    }

    return this.queueTransaction(transaction, description);
  }

  /**
   * Logs an action to blockchain
   * @param logData - Action log data
   * @returns Log result
   */
  public async logAction(logData: BlockchainLog): Promise<TransactionResult> {
    this.validateConnection();

    const logId = uuidv4();
    const logMessage = JSON.stringify({
      ...logData,
      logId,
      version: '1.0',
      sdk: '@synapserobo/sdk',
    });

    // Create memo instruction
    const memoInstruction = new TransactionInstruction({
      keys: [],
      programId: new PublicKey('MemoSq4gqABAXKb96qnH8TysNcWxMyWCqXgDLGmfcHr'),
      data: Buffer.from(logMessage, 'utf8'),
    });

    logger.debug('Logging action to blockchain', { logId, actionType: logData.type });

    return this.sendTransaction([memoInstruction], `Log action: ${logData.type}`);
  }

  /**
   * Processes a micro-payment
   * @param request - Payment request
   * @returns Payment result
   */
  public async processPayment(request: PaymentRequest): Promise<PaymentResult> {
    this.validateConnection();
    
    if (!this.config.microPaymentEnabled) {
      throw new Error('Micro-payments disabled');
    }

    if (!this.wallet) {
      throw new Error('Wallet not connected');
    }

    const recipient = request.recipient || this.config.paymentRecipient;
    if (!recipient) {
      throw new Error('No payment recipient configured');
    }

    const amount = request.amount || this.config.paymentAmount!;
    const lamports = Math.floor(amount * LAMPORTS_PER_SOL);

    const transferInstruction = SystemProgram.transfer({
      fromPubkey: this.wallet.publicKey,
      toPubkey: new PublicKey(recipient),
      lamports,
    });

    const description = `Micro-payment: ${amount} SOL to ${recipient}`;
    
    try {
      const result = await this.sendTransaction([transferInstruction], description);
      
      const paymentResult: PaymentResult = {
        success: result.success,
        signature: result.signature!,
        amount,
        recipient,
        timestamp: Date.now(),
        transactionId: result.signature,
      };

      logger.info('Micro-payment processed', paymentResult);
      return paymentResult;
    } catch (error) {
      logger.error('Payment failed', { error: error as Error, amount, recipient });
      throw error;
    }
  }

  /**
   * Gets wallet balance
   * @param publicKey - Optional public key
   * @returns Balance in lamports
   */
  public async getBalance(publicKey?: PublicKey): Promise<number> {
    this.validateConnection();
    
    const pubkey = publicKey || this.wallet!.publicKey;
    const balance = await this.connection!.getBalance(pubkey);
    
    logger.debug('Balance checked', { 
      publicKey: pubkey.toBase58(), 
      balance: balance / LAMPORTS_PER_SOL 
    });

    return balance;
  }

  /**
   * Gets transaction history for wallet
   * @param limit - Number of transactions
   * @returns Transaction signatures
   */
  public async getTransactionHistory(limit: number = 10): Promise<ConfirmedSignatureInfo[]> {
    this.validateConnection();
    
    if (!this.wallet) {
      throw new Error('Wallet not connected');
    }

    const signatures = await this.connection!.getConfirmedSignaturesForAddress2(
      this.wallet.publicKey,
      { limit }
    );

    logger.debug('Transaction history retrieved', { count: signatures.length });
    return signatures;
  }

  /**
   * Gets current wallet status
   */
  public getWalletStatus(): WalletStatus {
    return this.statusSubject.value;
  }

  /**
   * Gets network status
   */
  public getNetworkStatus(): NetworkStatus {
    return { ...this.networkStatus };
  }

  /**
   * Subscribes to Solana events
   */
  public getEventStream(): Observable<any> {
    return this.eventSubject.asObservable();
  }

  /**
   * Subscribes to wallet status changes
   */
  public getStatusStream(): Observable<WalletStatus> {
    return this.statusSubject.asObservable();
  }

  private validateConnection(): void {
    if (!this.connection) {
      throw new Error('Solana client not connected. Call connect() first.');
    }
  }

  private getDefaultEndpoint(network: string): string {
    const endpoints: Record<string, string> = {
      'mainnet-beta': 'https://api.mainnet-beta.solana.com',
      'testnet': 'https://api.testnet.solana.com',
      'devnet': 'https://api.devnet.solana.com',
    };

    return endpoints[network] || endpoints['devnet'];
  }

  private startNetworkMonitor(): void {
    interval(10000).pipe(
      switchMap(() => from(this.checkNetworkHealth())),
      catchError(() => of(false))
    ).subscribe(healthy => {
      this.networkStatus.healthy = healthy;
      this.networkStatus.lastCheck = Date.now();
      
      if (!healthy) {
        logger.warn('Solana network unhealthy');
      }
    });
  }

  private async checkNetworkHealth(): Promise<boolean> {
    if (!this.connection) return false;

    const start = Date.now();
    try {
      await this.connection.getSlot();
      this.networkStatus.latency = Date.now() - start;
      return this.networkStatus.latency < 5000;
    } catch {
      return false;
    }
  }

  private startTransactionProcessor(): void {
    interval(100).pipe(
      filter(() => this.transactionQueue.length > 0 && !this.processingQueue),
      tap(() => this.processTransactionQueue())
    ).subscribe();
  }

  private async processTransactionQueue(): Promise<void> {
    if (this.processingQueue || !this.connection || !this.wallet) return;

    this.processingQueue = true;
    const item = this.transactionQueue[0];

    try {
      // Sign transaction
      const signed = await this.signTransaction(item.transaction);
      
      // Send transaction
      const signature = await this.connection.sendRawTransaction(signed.serialize(), {
        skipPreflight: this.config.simulationMode,
        maxRetries: 0,
      });

      // Track confirmation
      this.trackConfirmation(signature, item);
      
      // Remove from queue
      this.transactionQueue.shift();

      logger.debug('Transaction sent', { 
        signature, 
        description: item.description,
        queueLength: this.transactionQueue.length 
      });

    } catch (error) {
      item.retries++;
      
      if (item.retries >= this.config.maxRetries!) {
        this.transactionQueue.shift();
        this.emitTransactionError(item, error as Error);
        logger.error('Transaction failed after retries', { 
          id: item.id, 
          error: error as Error 
        });
      } else {
        // Requeue with delay
        setTimeout(() => {
          const index = this.transactionQueue.indexOf(item);
          if (index > -1) {
            this.transactionQueue.splice(index, 1);
            this.transactionQueue.push(item);
          }
        }, this.config.retryDelay!);
      }
    } finally {
      this.processingQueue = false;
    }
  }

  private async signTransaction(transaction: Transaction): Promise<Transaction> {
    if (!this.wallet) throw new Error('Wallet not available');
    
    transaction.partialSign(this.wallet);
    return transaction;
  }

  private queueTransaction(transaction: Transaction, description: string): Promise<TransactionResult> {
    return new Promise((resolve, reject) => {
      const item: TransactionQueueItem = {
        transaction,
        description,
        retries: 0,
        timestamp: Date.now(),
        id: uuidv4(),
      };

      this.transactionQueue.push(item);

      // Store pending resolution
      const timeout = setTimeout(() => {
        this.pendingSignatures.delete(item.id);
        reject(new Error('Transaction timeout'));
      }, this.config.confirmTimeout!);

      this.pendingSignatures.set(item.id, {
        signature: '',
        resolve,
        reject,
        timeout,
      });
    });
  }

  private trackConfirmation(signature: TransactionSignature, queueItem: TransactionQueueItem): void {
    const pending = this.pendingSignatures.get(queueItem.id);
    if (!pending) return;

    pending.signature = signature;

    // Confirm transaction
    this.connection!.confirmTransaction(signature, this.config.commitment!)
      .then(result => {
        clearTimeout(pending.timeout);
        this.pendingSignatures.delete(queueItem.id);

        const txResult: TransactionResult = {
          success: !result.value.err,
          signature,
          timestamp: Date.now(),
          error: result.value.err ? JSON.stringify(result.value.err) : undefined,
        };

        pending.resolve(txResult);
        this.eventSubject.next({ type: 'transaction_confirmed', data: txResult });
      })
      .catch(error => {
        clearTimeout(pending.timeout);
        this.pendingSignatures.delete(queueItem.id);
        pending.reject(error);
      });
  }

  private async simulateTransaction(transaction: Transaction, description: string): Promise<TransactionResult> {
    await this.delay(500 + Math.random() * 1000);
    
    const signature = 'simulated_' + uuidv4().slice(0, 8);
    
    logger.debug('Transaction simulated', { signature, description });

    const result: TransactionResult = {
      success: true,
      signature,
      timestamp: Date.now(),
    };

    this.eventSubject.next({ type: 'transaction_simulated', data: result });
    return result;
  }

  private emitTransactionError(item: TransactionQueueItem, error: Error): void {
    const pending = this.pendingSignatures.get(item.id);
    if (pending) {
      clearTimeout(pending.timeout);
      this.pendingSignatures.delete(item.id);
      pending.reject(error);
    }

    this.eventSubject.next({ 
      type: 'transaction_error', 
      data: { item, error: error.message } 
    });
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}