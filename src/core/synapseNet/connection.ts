import { EventEmitter } from 'events';

// Connection configuration interface
export interface ConnectionConfig {
  endpoint: string;
  maxRetries: number;
  retryDelay: number;
  connectionTimeout: number;
  autoReconnect: boolean;
  reconnectDelay: number;
  maxReconnectAttempts: number;
  backoffMultiplier: number;
  heartbeatEnabled: boolean;
}

// Connection state enumeration
export enum ConnectionState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  RECONNECTING = 'reconnecting',
  DISCONNECTING = 'disconnecting',
  ERROR = 'error'
}

// Connection statistics
interface ConnectionStats {
  totalConnections: number;
  successfulConnections: number;
  failedConnections: number;
  totalUptime: number;
  lastConnectionTime: number;
  averageConnectionTime: number;
  retryCount: number;
}

// Retry strategy configuration
interface RetryStrategy {
  type: 'linear' | 'exponential' | 'fibonacci';
  baseDelay: number;
  maxDelay: number;
  maxAttempts: number;
}

// Main ConnectionManager class handling all connection logic
export class ConnectionManager {
  private config: ConnectionConfig;
  private currentState: ConnectionState;
  private eventEmitter: EventEmitter;
  private retryCount: number;
  private reconnectAttempts: number;
  private connectionStartTime: number | null;
  private lastConnectedTime: number | null;
  private stats: ConnectionStats;
  private retryStrategy: RetryStrategy;
  private connectionTimeoutId: NodeJS.Timeout | null;
  private reconnectIntervalId: NodeJS.Timeout | null;
  private validationIntervalId: NodeJS.Timeout | null;
  private isValidationActive: boolean;
  private connectionId: string | null;
  private endpointResolver: ((endpoint: string) => string) | null;
  private authToken: string | null;
  private connectionMetadata: Map<string, any>;

  constructor(config: ConnectionConfig) {
    this.config = {
      maxReconnectAttempts: 10,
      reconnectDelay: 5000,
      backoffMultiplier: 1.5,
      heartbeatEnabled: true,
      ...config
    };
    this.currentState = ConnectionState.DISCONNECTED;
    this.eventEmitter = new EventEmitter();
    this.retryCount = 0;
    this.reconnectAttempts = 0;
    this.connectionStartTime = null;
    this.lastConnectedTime = null;
    this.stats = {
      totalConnections: 0,
      successfulConnections: 0,
      failedConnections: 0,
      totalUptime: 0,
      lastConnectionTime: 0,
      averageConnectionTime: 0,
      retryCount: 0
    };
    this.retryStrategy = {
      type: 'exponential',
      baseDelay: this.config.retryDelay,
      maxDelay: 30000,
      maxAttempts: this.config.maxRetries
    };
    this.connectionTimeoutId = null;
    this.reconnectIntervalId = null;
    this.validationIntervalId = null;
    this.isValidationActive = false;
    this.connectionId = null;
    this.endpointResolver = null;
    this.authToken = null;
    this.connectionMetadata = new Map();
    this.setupEventForwarding();
  }

  // Initialize connection manager
  public async initialize(): Promise<boolean> {
    try {
      this.updateState(ConnectionState.DISCONNECTED);
      this.connectionMetadata.clear();
      this.eventEmitter.emit('initialized', { timestamp: Date.now(), config: this.config });
      return true;
    } catch (error) {
      this.eventEmitter.emit('error', { error, context: 'initialization', timestamp: Date.now() });
      return false;
    }
  }

  // Main connection method with full handshake
  public async connect(): Promise<boolean> {
    if (this.currentState === ConnectionState.CONNECTED) {
      return true;
    }
    
    if (this.currentState === ConnectionState.CONNECTING || 
        this.currentState === ConnectionState.RECONNECTING) {
      return false;
    }
    
    this.connectionStartTime = Date.now();
    this.updateState(ConnectionState.CONNECTING);
    this.connectionId = this.generateConnectionId();
    
    try {
      const resolvedEndpoint = this.resolveEndpoint(this.config.endpoint);
      const connectionResult = await this.establishConnection(resolvedEndpoint);
      
      if (connectionResult) {
        this.handleSuccessfulConnection();
        return true;
      } else {
        this.handleFailedConnection(new Error('Connection establishment failed'));
        return false;
      }
    } catch (error) {
      this.handleFailedConnection(error as Error);
      return false;
    }
  }

  public async disconnect(): Promise<boolean> {
    if (this.currentState === ConnectionState.DISCONNECTED) {
      return true;
    }
    
    this.updateState(ConnectionState.DISCONNECTING);
    
    try {
      await this.performDisconnection();
      this.handleDisconnection();
      return true;
    } catch (error) {
      this.eventEmitter.emit('error', { error, context: 'disconnection', timestamp: Date.now() });
      this.forceDisconnection();
      return false;
    }
  }

  public async reconnect(): Promise<boolean> {
    if (this.currentState === ConnectionState.RECONNECTING) {
      return false;
    }
    
    if (!this.config.autoReconnect) {
      return false;
    }
    
    this.updateState(ConnectionState.RECONNECTING);
    this.reconnectAttempts = 0;
    
    const success = await this.attemptReconnection();
    
    if (success) {
      this.updateState(ConnectionState.CONNECTED);
      this.reconnectAttempts = 0;
      this.eventEmitter.emit('reconnected', { 
        attempts: this.reconnectAttempts, 
        timestamp: Date.now() 
      });
    }
    
    return success;
  }

  // Validate current connection health
  public async validateConnection(): Promise<{ valid: boolean; latency?: number; details?: any }> {
    if (this.currentState !== ConnectionState.CONNECTED) {
      return { valid: false, details: { state: this.currentState } };
    }
    
    try {
      const startTime = Date.now();
      const validationResult = await this.performValidation();
      const latency = Date.now() - startTime;
      
      if (validationResult) {
        this.eventEmitter.emit('validation_passed', { latency, timestamp: Date.now() });
        return { valid: true, latency };
      } else {
        this.eventEmitter.emit('validation_failed', { latency, timestamp: Date.now() });
        return { valid: false, latency };
      }
    } catch (error) {
      this.eventEmitter.emit('validation_error', { error, timestamp: Date.now() });
      return { valid: false, details: { error } };
    }
  }

  // Start periodic connection validation
  public startValidation(interval: number = 30000): void {
    if (this.validationIntervalId) {
      clearInterval(this.validationIntervalId);
    }
    
    this.isValidationActive = true;
    this.validationIntervalId = setInterval(async () => {
      if (this.currentState === ConnectionState.CONNECTED) {
        await this.validateConnection();
      }
    }, interval);
    
    this.eventEmitter.emit('validation_started', { interval, timestamp: Date.now() });
  }

  // Stop periodic connection validation
  public stopValidation(): void {
    if (this.validationIntervalId) {
      clearInterval(this.validationIntervalId);
      this.validationIntervalId = null;
    }
    
    this.isValidationActive = false;
    this.eventEmitter.emit('validation_stopped', { timestamp: Date.now() });
  }

  // Update configuration dynamically
  public updateConfig(newConfig: Partial<ConnectionConfig>): void {
    const oldConfig = { ...this.config };
    this.config = { ...this.config, ...newConfig };
    
    if (newConfig.endpoint && newConfig.endpoint !== oldConfig.endpoint) {
      this.eventEmitter.emit('endpoint_changed', { 
        old: oldConfig.endpoint, 
        new: newConfig.endpoint,
        timestamp: Date.now() 
      });
    }
    
    this.eventEmitter.emit('config_updated', { 
      oldConfig, 
      newConfig: this.config, 
      timestamp: Date.now() 
    });
  }

  // Set authentication token for connection
  public setAuthToken(token: string): void {
    this.authToken = token;
    this.connectionMetadata.set('authTokenSet', true);
    this.eventEmitter.emit('auth_token_set', { timestamp: Date.now() });
  }

  public setEndpointResolver(resolver: (endpoint: string) => string): void {
    this.endpointResolver = resolver;
  }

  public getState(): ConnectionState {
    return this.currentState;
  }

  public isConnected(): boolean {
    return this.currentState === ConnectionState.CONNECTED;
  }

  public getStats(): ConnectionStats {
    return { ...this.stats };
  }

  // Get connection metadata
  public getMetadata(key?: string): any {
    if (key) {
      return this.connectionMetadata.get(key);
    }
    return Object.fromEntries(this.connectionMetadata);
  }

  public setMetadata(key: string, value: any): void {
    this.connectionMetadata.set(key, value);
  }

  public onConnected(handler: Function): void {
    this.eventEmitter.on('connected', handler);
  }

  public onDisconnected(handler: Function): void {
    this.eventEmitter.on('disconnected', handler);
  }

  public onReconnecting(handler: Function): void {
    this.eventEmitter.on('reconnecting', handler);
  }

  public onError(handler: Function): void {
    this.eventEmitter.on('error', handler);
  }

  private updateState(newState: ConnectionState): void {
    const oldState = this.currentState;
    this.currentState = newState;
    
    this.eventEmitter.emit('state_changed', { 
      oldState, 
      newState, 
      timestamp: Date.now(),
      connectionId: this.connectionId
    });
  }

  private resolveEndpoint(endpoint: string): string {
    if (this.endpointResolver) {
      return this.endpointResolver(endpoint);
    }
    return endpoint;
  }

  // Private method to establish connection
  private async establishConnection(endpoint: string): Promise<boolean> {
    this.setConnectionTimeout();
    
    try {
      console.log(`Connecting to ${endpoint}`);
      
      await this.simulateConnection();
      
      if (this.connectionTimeoutId) {
        clearTimeout(this.connectionTimeoutId);
        this.connectionTimeoutId = null;
      }
      
      return true;
    } catch (error) {
      if (this.connectionTimeoutId) {
        clearTimeout(this.connectionTimeoutId);
        this.connectionTimeoutId = null;
      }
      throw error;
    }
  }

  // Simulate connection for testing/placeholder
  private async simulateConnection(): Promise<void> {
    return new Promise((resolve, reject) => {
      setTimeout(() => {
        if (Math.random() > 0.1) { 
          resolve();
        } else {
          reject(new Error('Simulated connection failure'));
        }
      }, 500);
    });
  }

  private setConnectionTimeout(): void {
    this.connectionTimeoutId = setTimeout(() => {
      this.eventEmitter.emit('connection_timeout', { 
        timeout: this.config.connectionTimeout,
        timestamp: Date.now() 
      });
      this.handleFailedConnection(new Error('Connection timeout'));
    }, this.config.connectionTimeout);
  }

  private handleSuccessfulConnection(): void {
    this.updateState(ConnectionState.CONNECTED);
    this.lastConnectedTime = Date.now();
    this.retryCount = 0;
    this.reconnectAttempts = 0;
    
    this.stats.totalConnections++;
    this.stats.successfulConnections++;
    this.stats.lastConnectionTime = Date.now();
    
    if (this.connectionStartTime) {
      const connectionTime = Date.now() - this.connectionStartTime;
      this.stats.totalUptime += connectionTime;
      this.stats.averageConnectionTime = this.stats.totalUptime / this.stats.successfulConnections;
    }
    
    this.startHealthMonitoring();
    this.eventEmitter.emit('connected', { 
      connectionId: this.connectionId,
      timestamp: Date.now(),
      stats: { ...this.stats }
    });
  }

  // Handle failed connection
  private handleFailedConnection(error: Error): void {
    this.updateState(ConnectionState.ERROR);
    this.stats.totalConnections++;
    this.stats.failedConnections++;
    this.stats.retryCount = this.retryCount;
    
    this.eventEmitter.emit('connection_failed', { 
      error, 
      retryCount: this.retryCount,
      timestamp: Date.now() 
    });
    
    if (this.retryCount < this.config.maxRetries && this.config.autoReconnect) {
      this.scheduleRetry();
    } else {
      this.updateState(ConnectionState.DISCONNECTED);
      this.eventEmitter.emit('disconnected', { 
        reason: 'max_retries_exceeded',
        timestamp: Date.now() 
      });
    }
  }

  // Handle disconnection
  private handleDisconnection(): void {
    this.updateState(ConnectionState.DISCONNECTED);
    
    if (this.connectionStartTime && this.lastConnectedTime) {
      const uptime = this.lastConnectedTime - this.connectionStartTime;
      this.stats.totalUptime += uptime;
    }
    
    this.cleanupConnection();
    this.eventEmitter.emit('disconnected', { 
      connectionId: this.connectionId,
      timestamp: Date.now(),
      reason: 'graceful'
    });
    
    this.connectionId = null;
  }

  // Force disconnection without cleanup
  private forceDisconnection(): void {
    this.updateState(ConnectionState.DISCONNECTED);
    this.cleanupConnection();
    this.eventEmitter.emit('disconnected', { 
      connectionId: this.connectionId,
      timestamp: Date.now(),
      reason: 'forced'
    });
    
    this.connectionId = null;
  }

  // Attempt reconnection with backoff strategy
  private async attemptReconnection(): Promise<boolean> {
    while (this.reconnectAttempts < this.config.maxReconnectAttempts) {
      this.reconnectAttempts++;
      
      const delay = this.calculateReconnectDelay();
      this.eventEmitter.emit('reconnecting', { 
        attempt: this.reconnectAttempts,
        maxAttempts: this.config.maxReconnectAttempts,
        delay,
        timestamp: Date.now() 
      });
      
      await this.delay(delay);
      
      try {
        const success = await this.connect();
        if (success) {
          return true;
        }
      } catch (error) {
        // Continue to next attempt
      }
    }
    
    this.updateState(ConnectionState.DISCONNECTED);
    this.eventEmitter.emit('reconnection_failed', { 
      attempts: this.reconnectAttempts,
      timestamp: Date.now() 
    });
    
    return false;
  }

  // Calculate reconnect delay with backoff
  private calculateReconnectDelay(): number {
    if (this.retryStrategy.type === 'linear') {
      return Math.min(
        this.retryStrategy.baseDelay * this.reconnectAttempts,
        this.retryStrategy.maxDelay
      );
    } else if (this.retryStrategy.type === 'exponential') {
      return Math.min(
        this.retryStrategy.baseDelay * Math.pow(this.config.backoffMultiplier, this.reconnectAttempts - 1),
        this.retryStrategy.maxDelay
      );
    } else { // fibonacci
      let a = 0, b = this.retryStrategy.baseDelay;
      for (let i = 1; i < this.reconnectAttempts; i++) {
        [a, b] = [b, a + b];
      }
      return Math.min(b, this.retryStrategy.maxDelay);
    }
  }

  // Schedule retry with backoff
  private scheduleRetry(): void {
    this.retryCount++;
    const delay = this.calculateRetryDelay();
    
    setTimeout(() => {
      this.connect();
    }, delay);
    
    this.eventEmitter.emit('retry_scheduled', { 
      retryCount: this.retryCount,
      delay,
      timestamp: Date.now() 
    });
  }

  // Calculate retry delay
  private calculateRetryDelay(): number {
    return Math.min(
      this.config.retryDelay * Math.pow(2, this.retryCount - 1),
      30000
    );
  }

  // Perform disconnection (placeholder)
  private async performDisconnection(): Promise<void> {
    return new Promise((resolve) => {
      setTimeout(() => {
        console.log('Disconnected from endpoint');
        resolve();
      }, 200);
    });
  }

  // Perform connection validation (placeholder)
  private async performValidation(): Promise<boolean> {
    return new Promise((resolve) => {
      setTimeout(() => {
        resolve(Math.random() > 0.1); 
      }, 100);
    });
  }

  // Start health monitoring
  private startHealthMonitoring(): void {
    if (this.config.heartbeatEnabled) {
      this.startValidation(30000);
    }
  }

  // Cleanup connection resources
  private cleanupConnection(): void {
    this.stopValidation();
    
    if (this.connectionTimeoutId) {
      clearTimeout(this.connectionTimeoutId);
      this.connectionTimeoutId = null;
    }
    
    if (this.reconnectIntervalId) {
      clearInterval(this.reconnectIntervalId);
      this.reconnectIntervalId = null;
    }
    
    this.authToken = null;
    this.connectionMetadata.clear();
  }

  // Generate unique connection ID
  private generateConnectionId(): string {
    return `conn_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  // Setup event forwarding to parent
  private setupEventForwarding(): void {
    const eventsToForward = ['connected', 'disconnected', 'reconnecting', 'error'];
    eventsToForward.forEach(event => {
      this.eventEmitter.on(event, (data) => {
        this.eventEmitter.emit(event, data);
      });
    });
  }

  // Utility delay function
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  // Set custom retry strategy
  public setRetryStrategy(strategy: Partial<RetryStrategy>): void {
    this.retryStrategy = { ...this.retryStrategy, ...strategy };
    this.eventEmitter.emit('retry_strategy_updated', { 
      strategy: this.retryStrategy,
      timestamp: Date.now() 
    });
  }

  // Reset connection statistics
  public resetStats(): void {
    this.stats = {
      totalConnections: 0,
      successfulConnections: 0,
      failedConnections: 0,
      totalUptime: 0,
      lastConnectionTime: 0,
      averageConnectionTime: 0,
      retryCount: 0
    };
    this.eventEmitter.emit('stats_reset', { timestamp: Date.now() });
  }
}