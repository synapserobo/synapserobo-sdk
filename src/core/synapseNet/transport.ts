import { EventEmitter } from 'events';

// Core message interfaces for SynapseNet communication
export interface TelemetryMessage {
  deviceId: string;
  timestamp: number;
  metrics: {
    [key: string]: number | string | boolean;
  };
  location?: {
    lat: number;
    lng: number;
    accuracy?: number;
  };
  status: 'active' | 'idle' | 'error' | 'maintenance';
  batteryLevel?: number;
  signalStrength?: number;
  sequenceNumber: number;
}

export interface TaskMessage {
  taskId: string;
  command: string;
  parameters: {
    [key: string]: any;
  };
  priority: number;
  timeout?: number;
  source: string;
  destination: string;
  createdAt: number;
  requiresConfirmation: boolean;
}

export interface DeviceStateMessage {
  deviceId: string;
  state: 'online' | 'offline' | 'busy' | 'error';
  capabilities: string[];
  currentTask?: string | null;
  lastUpdated: number;
  metadata?: {
    [key: string]: any;
  };
}

// Internal message wrapper for routing
interface InternalMessageWrapper {
  id: string;
  type: 'telemetry' | 'task' | 'state' | 'control';
  payload: any;
  timestamp: number;
  source: string;
  destination?: string;
  retryCount: number;
  maxRetries: number;
}

// Transport configuration interface
interface TransportConfig {
  endpoint: string;
  telemetryPath: string;
  taskPath: string;
  statePath: string;
  maxMessageSize?: number;
  defaultTimeout?: number;
  enableCompression?: boolean;
}

// Internal routing table entry
interface RouteEntry {
  pattern: string;
  handler: (message: any) => Promise<void>;
  priority: number;
}

// Main TransportManager class handling all message routing and delivery
export class TransportManager {
  private config: TransportConfig;
  private routes: Map<string, RouteEntry[]>;
  private messageQueue: InternalMessageWrapper[];
  private isConnected: boolean;
  private eventEmitter: EventEmitter;
  private sequenceCounters: Map<string, number>;
  private pendingAcks: Map<string, NodeJS.Timeout>;
  private healthStatus: boolean;
  private lastError: Error | null;
  private compressionEnabled: boolean;
  private messageValidators: Map<string, (msg: any) => boolean>;

  constructor(config: TransportConfig) {
    this.config = {
      maxMessageSize: 1024 * 1024, // 1MB
      defaultTimeout: 10000,
      enableCompression: true,
      ...config
    };
    this.routes = new Map();
    this.messageQueue = [];
    this.isConnected = false;
    this.eventEmitter = new EventEmitter();
    this.sequenceCounters = new Map();
    this.pendingAcks = new Map();
    this.healthStatus = true;
    this.lastError = null;
    this.compressionEnabled = this.config.enableCompression!;
    this.messageValidators = new Map();
    this.setupDefaultValidators();
    this.startQueueProcessor();
  }

  // Initialize transport layer and setup routing
  public initialize(): void {
    this.setupDefaultRoutes();
    this.isConnected = true;
    this.healthStatus = true;
    this.eventEmitter.emit('initialized', { timestamp: Date.now() });
  }

  // Setup default message validators for each message type
  private setupDefaultValidators(): void {
    this.messageValidators.set('telemetry', this.validateTelemetryMessage.bind(this));
    this.messageValidators.set('task', this.validateTaskMessage.bind(this));
    this.messageValidators.set('state', this.validateStateMessage.bind(this));
  }

  // Validate telemetry message structure and content
  private validateTelemetryMessage(msg: TelemetryMessage): boolean {
    if (!msg.deviceId || typeof msg.deviceId !== 'string') return false;
    if (!msg.timestamp || typeof msg.timestamp !== 'number') return false;
    if (!msg.metrics || typeof msg.metrics !== 'object') return false;
    if (!msg.status || !['active', 'idle', 'error', 'maintenance'].includes(msg.status)) return false;
    if (msg.sequenceNumber === undefined || typeof msg.sequenceNumber !== 'number') return false;
    if (msg.location) {
      if (typeof msg.location.lat !== 'number' || typeof msg.location.lng !== 'number') return false;
    }
    return true;
  }

  // Validate task message structure and content
  private validateTaskMessage(msg: TaskMessage): boolean {
    if (!msg.taskId || typeof msg.taskId !== 'string') return false;
    if (!msg.command || typeof msg.command !== 'string') return false;
    if (!msg.parameters || typeof msg.parameters !== 'object') return false;
    if (!msg.source || typeof msg.source !== 'string') return false;
    if (!msg.destination || typeof msg.destination !== 'string') return false;
    if (!msg.createdAt || typeof msg.createdAt !== 'number') return false;
    if (msg.priority === undefined || typeof msg.priority !== 'number') return false;
    return true;
  }

  // Validate device state message structure and content
  private validateStateMessage(msg: DeviceStateMessage): boolean {
    if (!msg.deviceId || typeof msg.deviceId !== 'string') return false;
    if (!msg.state || !['online', 'offline', 'busy', 'error'].includes(msg.state)) return false;
    if (!msg.capabilities || !Array.isArray(msg.capabilities)) return false;
    if (!msg.lastUpdated || typeof msg.lastUpdated !== 'number') return false;
    return true;
  }

  // Setup default routing paths for different message types
  private setupDefaultRoutes(): void {
    this.addRoute('telemetry.*', async (msg) => {
      await this.handleTelemetry(msg);
    }, 1);
    
    this.addRoute('task.*', async (msg) => {
      await this.handleTask(msg);
    }, 2);
    
    this.addRoute('state.*', async (msg) => {
      await this.handleState(msg);
    }, 1);
    
    this.addRoute('control.heartbeat', async (msg) => {
      await this.handleHeartbeat(msg);
    }, 0);
  }

  // Add a new routing rule with pattern matching
  public addRoute(pattern: string, handler: (message: any) => Promise<void>, priority: number = 1): void {
    const routeEntry: RouteEntry = {
      pattern,
      handler,
      priority
    };
    
    if (!this.routes.has(pattern)) {
      this.routes.set(pattern, []);
    }
    
    this.routes.get(pattern)!.push(routeEntry);
    this.routes.get(pattern)!.sort((a, b) => b.priority - a.priority);
  }

  // Send telemetry data with validation and routing
  public async sendTelemetry(data: TelemetryMessage): Promise<string> {
    const validator = this.messageValidators.get('telemetry');
    if (!validator || !validator(data)) {
      throw new Error('Invalid telemetry message format');
    }
    
    const messageId = this.generateMessageId('telemetry');
    const wrappedMessage: InternalMessageWrapper = {
      id: messageId,
      type: 'telemetry',
      payload: data,
      timestamp: Date.now(),
      source: data.deviceId,
      retryCount: 0,
      maxRetries: 3
    };
    
    return await this.dispatch(wrappedMessage);
  }

  // Send task command with validation and routing
  public async sendTask(data: TaskMessage): Promise<string> {
    const validator = this.messageValidators.get('task');
    if (!validator || !validator(data)) {
      throw new Error('Invalid task message format');
    }
    
    const messageId = this.generateMessageId('task');
    const wrappedMessage: InternalMessageWrapper = {
      id: messageId,
      type: 'task',
      payload: data,
      timestamp: Date.now(),
      source: data.source,
      destination: data.destination,
      retryCount: 0,
      maxRetries: 5
    };
    
    return await this.dispatch(wrappedMessage);
  }

  // Send device state update with validation and routing
  public async sendDeviceState(data: DeviceStateMessage): Promise<string> {
    const validator = this.messageValidators.get('state');
    if (!validator || !validator(data)) {
      throw new Error('Invalid state message format');
    }
    
    const messageId = this.generateMessageId('state');
    const wrappedMessage: InternalMessageWrapper = {
      id: messageId,
      type: 'state',
      payload: data,
      timestamp: Date.now(),
      source: data.deviceId,
      retryCount: 0,
      maxRetries: 3
    };
    
    return await this.dispatch(wrappedMessage);
  }

  // Main dispatch method for routing messages
  private async dispatch(message: InternalMessageWrapper): Promise<string> {
    if (!this.isConnected) {
      this.queueMessage(message);
      return 'queued';
    }
    
    if (this.isMessageTooLarge(message)) {
      throw new Error(`Message exceeds maximum size of ${this.config.maxMessageSize} bytes`);
    }
    
    try {
      const routeKey = `${message.type}.${message.destination || '*'}`;
      const routes = this.findMatchingRoutes(routeKey);
      
      if (routes.length === 0) {
        throw new Error(`No route found for message type: ${message.type}`);
      }
      
      for (const route of routes) {
        await route.handler(message);
      }
      
      this.setupAckTimeout(message);
      return message.id;
    } catch (error) {
      this.lastError = error as Error;
      this.retryMessage(message);
      throw error;
    }
  }

  // Find all routes matching the given pattern
  private findMatchingRoutes(pattern: string): RouteEntry[] {
    const matchingRoutes: RouteEntry[] = [];
    
    for (const [routePattern, routes] of this.routes.entries()) {
      if (this.patternMatches(routePattern, pattern)) {
        matchingRoutes.push(...routes);
      }
    }
    
    return matchingRoutes.sort((a, b) => b.priority - a.priority);
  }

  // Check if route pattern matches message pattern
  private patternMatches(routePattern: string, messagePattern: string): boolean {
    if (routePattern === messagePattern) return true;
    if (routePattern.endsWith('.*')) {
      const routeBase = routePattern.slice(0, -2);
      return messagePattern.startsWith(routeBase);
    }
    return false;
  }

  // Handle telemetry message routing and processing
  private async handleTelemetry(message: InternalMessageWrapper): Promise<void> {
    const telemetry = message.payload as TelemetryMessage;
    const endpoint = `${this.config.endpoint}${this.config.telemetryPath}`;
    
    const processedData = this.processTelemetryData(telemetry);
    await this.sendToEndpoint(endpoint, processedData);
    
    this.eventEmitter.emit('telemetry_sent', {
      messageId: message.id,
      deviceId: telemetry.deviceId,
      timestamp: Date.now()
    });
  }

  // Handle task message routing and processing
  private async handleTask(message: InternalMessageWrapper): Promise<void> {
    const task = message.payload as TaskMessage;
    const endpoint = `${this.config.endpoint}${this.config.taskPath}`;
    
    await this.sendToEndpoint(endpoint, task);
    
    this.eventEmitter.emit('task_sent', {
      messageId: message.id,
      taskId: task.taskId,
      timestamp: Date.now()
    });
  }

  // Handle state message routing and processing
  private async handleState(message: InternalMessageWrapper): Promise<void> {
    const state = message.payload as DeviceStateMessage;
    const endpoint = `${this.config.endpoint}${this.config.statePath}`;
    
    await this.sendToEndpoint(endpoint, state);
    
    this.eventEmitter.emit('state_sent', {
      messageId: message.id,
      deviceId: state.deviceId,
      timestamp: Date.now()
    });
  }

  // Handle heartbeat control messages
  private async handleHeartbeat(message: InternalMessageWrapper): Promise<void> {
    this.eventEmitter.emit('heartbeat_received', {
      messageId: message.id,
      timestamp: Date.now()
    });
  }

  // Process telemetry data before sending (compression, filtering, etc.)
  private processTelemetryData(data: TelemetryMessage): any {
    let processed = { ...data };
    
    if (this.compressionEnabled) {
      processed = this.compressTelemetryData(processed);
    }
    
    processed.metrics = this.filterMetrics(processed.metrics);
    return processed;
  }

  // Compress telemetry data to reduce bandwidth
  private compressTelemetryData(data: TelemetryMessage): TelemetryMessage {
    const compressed = { ...data };
    if (compressed.metrics) {
      for (const key in compressed.metrics) {
        if (typeof compressed.metrics[key] === 'number') {
          compressed.metrics[key] = parseFloat((compressed.metrics[key] as number).toFixed(3));
        }
      }
    }
    return compressed;
  }

  // Filter metrics to remove null/undefined values
  private filterMetrics(metrics: { [key: string]: any }): { [key: string]: any } {
    const filtered: { [key: string]: any } = {};
    for (const key in metrics) {
      if (metrics[key] !== null && metrics[key] !== undefined) {
        filtered[key] = metrics[key];
      }
    }
    return filtered;
  }

  // Send data to endpoint with error handling
  private async sendToEndpoint(endpoint: string, data: any): Promise<void> {
    const timeout = this.config.defaultTimeout!;
    
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), timeout);
    
    try {
      console.log(`Sending to ${endpoint}:`, data);
      
      clearTimeout(timeoutId);
    } catch (error) {
      clearTimeout(timeoutId);
      throw error;
    }
  }

  // Generate unique message ID
  private generateMessageId(type: string): string {
    const counter = (this.sequenceCounters.get(type) || 0) + 1;
    this.sequenceCounters.set(type, counter);
    return `${type}_${Date.now()}_${counter}`;
  }

  // Check if message exceeds size limit
  private isMessageTooLarge(message: InternalMessageWrapper): boolean {
    const messageSize = JSON.stringify(message).length;
    return messageSize > this.config.maxMessageSize!;
  }

  // Queue message for later processing
  private queueMessage(message: InternalMessageWrapper): void {
    this.messageQueue.push(message);
    if (this.messageQueue.length > 1000) {
      this.messageQueue.shift();
    }
  }

  // Start background queue processor
  private startQueueProcessor(): void {
    setInterval(() => {
      this.processQueue();
    }, 100);
  }

  // Process queued messages
  private async processQueue(): Promise<void> {
    if (!this.isConnected || this.messageQueue.length === 0) {
      return;
    }
    
    const message = this.messageQueue.shift();
    if (message) {
      try {
        await this.dispatch(message);
      } catch (error) {
        this.messageQueue.unshift(message);
      }
    }
  }

  // Setup acknowledgment timeout for message
  private setupAckTimeout(message: InternalMessageWrapper): void {
    const timeout = setTimeout(() => {
      if (this.pendingAcks.has(message.id)) {
        this.retryMessage(message);
        this.pendingAcks.delete(message.id);
      }
    }, 5000);
    
    this.pendingAcks.set(message.id, timeout);
  }

  // Retry message delivery with backoff
  private retryMessage(message: InternalMessageWrapper): void {
    if (message.retryCount >= message.maxRetries) {
      this.eventEmitter.emit('message_failed', {
        messageId: message.id,
        reason: 'max_retries_exceeded'
      });
      return;
    }
    
    message.retryCount++;
    const backoffDelay = Math.min(1000 * Math.pow(2, message.retryCount), 30000);
    
    setTimeout(() => {
      this.queueMessage(message);
    }, backoffDelay);
  }

  // Check transport health status
  public isHealthy(): boolean {
    return this.healthStatus && this.isConnected;
  }

  // Update connection status
  public setConnected(status: boolean): void {
    this.isConnected = status;
    this.healthStatus = status;
    if (status) {
      this.eventEmitter.emit('connected', { timestamp: Date.now() });
    } else {
      this.eventEmitter.emit('disconnected', { timestamp: Date.now() });
    }
  }

  // Register event listener
  public on(event: string, listener: Function): void {
    this.eventEmitter.on(event, listener);
  }
}