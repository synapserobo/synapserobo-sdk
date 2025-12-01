import { TransportManager, TelemetryMessage, TaskMessage, DeviceStateMessage } from './transport';
import { HeartbeatManager } from './heartbeat';
import { ConnectionManager } from './connection';

// Internal event types for the SynapseNet event bus
export enum SynapseEvent {
  CONNECTED = 'connected',
  DISCONNECTED = 'disconnected',
  TELEMETRY_SENT = 'telemetry_sent',
  TELEMETRY_RECEIVED = 'telemetry_received',
  TASK_SENT = 'task_sent',
  TASK_RECEIVED = 'task_received',
  STATE_UPDATE = 'state_update',
  HEARTBEAT_TIMEOUT = 'heartbeat_timeout',
  HEARTBEAT_RESTORED = 'heartbeat_restored',
  RECONNECTING = 'reconnecting',
  ERROR = 'error'
}

// Core SynapseNet configuration interface
export interface SynapseNetConfig {
  endpoint: string;
  telemetryPath?: string;
  taskPath?: string;
  statePath?: string;
  heartbeatInterval?: number;
  heartbeatTimeout?: number;
  maxRetries?: number;
  retryDelay?: number;
  connectionTimeout?: number;
  autoReconnect?: boolean;
}

// Internal event bus structure for message routing
interface InternalEvent {
  type: SynapseEvent;
  payload: any;
  timestamp: number;
  source?: string;
}

// Main SynapseNet class coordinating all subsystems
export class SynapseNet {
  private transport: TransportManager;
  private heartbeat: HeartbeatManager;
  private connection: ConnectionManager;
  private config: SynapseNetConfig;
  private internalState: SynapseNetState;
  private eventHandlers: Map<SynapseEvent, Function[]>;
  private messageQueue: Map<string, any[]>;
  private isInitialized: boolean;
  private sessionId: string | null;
  private lastActivity: number;
  private componentStatus: Map<string, boolean>;

  constructor(config: SynapseNetConfig) {
    this.config = {
      heartbeatInterval: 30000,
      heartbeatTimeout: 90000,
      maxRetries: 5,
      retryDelay: 1000,
      connectionTimeout: 10000,
      autoReconnect: true,
      telemetryPath: '/telemetry',
      taskPath: '/tasks',
      statePath: '/state',
      ...config
    };
    this.transport = new TransportManager(this.config);
    this.heartbeat = new HeartbeatManager();
    this.connection = new ConnectionManager(this.config);
    this.internalState = SynapseNetState.DISCONNECTED;
    this.eventHandlers = new Map();
    this.messageQueue = new Map();
    this.isInitialized = false;
    this.sessionId = null;
    this.lastActivity = Date.now();
    this.componentStatus = new Map();
    this.setupInternalListeners();
  }

  // Initialize all subsystems and prepare for connection
  public async initialize(): Promise<boolean> {
    if (this.isInitialized) {
      return true;
    }
    try {
      this.transport.initialize();
      this.heartbeat.initialize();
      const initResult = await this.connection.initialize();
      if (initResult) {
        this.isInitialized = true;
        this.emitInternalEvent(SynapseEvent.CONNECTED, { timestamp: Date.now() });
        this.startHealthMonitoring();
      }
      return initResult;
    } catch (error) {
      this.emitInternalEvent(SynapseEvent.ERROR, { error, context: 'initialization' });
      return false;
    }
  }

  // Main connection method with session establishment
  public async connect(): Promise<boolean> {
    if (!this.isInitialized) {
      const initialized = await this.initialize();
      if (!initialized) {
        return false;
      }
    }
    try {
      const connectionResult = await this.connection.connect();
      if (connectionResult) {
        this.internalState = SynapseNetState.CONNECTED;
        this.sessionId = this.generateSessionId();
        this.heartbeat.start(this.config.heartbeatInterval!);
        this.flushQueuedMessages();
        this.emitInternalEvent(SynapseEvent.CONNECTED, { 
          sessionId: this.sessionId, 
          timestamp: Date.now() 
        });
      }
      return connectionResult;
    } catch (error) {
      this.emitInternalEvent(SynapseEvent.ERROR, { error, context: 'connection' });
      return false;
    }
  }

  // Send telemetry data with automatic routing and validation
  public async sendTelemetry(data: TelemetryMessage): Promise<string> {
    this.updateLastActivity();
    if (this.internalState !== SynapseNetState.CONNECTED) {
      this.queueMessage('telemetry', data);
      return 'queued';
    }
    try {
      const result = await this.transport.sendTelemetry(data);
      this.emitInternalEvent(SynapseEvent.TELEMETRY_SENT, { data, timestamp: Date.now() });
      return result;
    } catch (error) {
      this.queueMessage('telemetry', data);
      this.emitInternalEvent(SynapseEvent.ERROR, { error, context: 'sendTelemetry' });
      throw error;
    }
  }

  // Send task commands with priority handling
  public async sendTask(task: TaskMessage, priority: number = 1): Promise<string> {
    this.updateLastActivity();
    if (this.internalState !== SynapseNetState.CONNECTED) {
      this.queueMessage('task', { task, priority });
      return 'queued';
    }
    try {
      const result = await this.transport.sendTask(task);
      this.emitInternalEvent(SynapseEvent.TASK_SENT, { task, priority, timestamp: Date.now() });
      return result;
    } catch (error) {
      this.queueMessage('task', { task, priority });
      this.emitInternalEvent(SynapseEvent.ERROR, { error, context: 'sendTask' });
      throw error;
    }
  }

  // Update device state with consistency checks
  public async updateDeviceState(state: DeviceStateMessage): Promise<string> {
    this.updateLastActivity();
    if (this.internalState !== SynapseNetState.CONNECTED) {
      this.queueMessage('state', state);
      return 'queued';
    }
    try {
      const result = await this.transport.sendDeviceState(state);
      this.emitInternalEvent(SynapseEvent.STATE_UPDATE, { state, timestamp: Date.now() });
      return result;
    } catch (error) {
      this.queueMessage('state', state);
      this.emitInternalEvent(SynapseEvent.ERROR, { error, context: 'updateDeviceState' });
      throw error;
    }
  }

  // Register message handler for incoming messages
  public onMessage(eventType: SynapseEvent, handler: Function): void {
    if (!this.eventHandlers.has(eventType)) {
      this.eventHandlers.set(eventType, []);
    }
    this.eventHandlers.get(eventType)!.push(handler);
  }

  // Internal event emission for component coordination
  private emitInternalEvent(eventType: SynapseEvent, payload: any): void {
    const event: InternalEvent = {
      type: eventType,
      payload,
      timestamp: Date.now(),
      source: 'synapseNet'
    };
    const handlers = this.eventHandlers.get(eventType) || [];
    handlers.forEach(handler => {
      try {
        handler(event);
      } catch (error) {
        console.error(`Error in event handler for ${eventType}:`, error);
      }
    });
  }

  // Queue messages when disconnected for later transmission
  private queueMessage(queueType: string, message: any): void {
    if (!this.messageQueue.has(queueType)) {
      this.messageQueue.set(queueType, []);
    }
    this.messageQueue.get(queueType)!.push(message);
    if (this.messageQueue.get(queueType)!.length > 1000) {
      this.messageQueue.get(queueType)!.shift();
    }
  }

  // Send all queued messages after reconnection
  private flushQueuedMessages(): void {
    for (const [queueType, messages] of this.messageQueue.entries()) {
      while (messages.length > 0) {
        const message = messages.shift();
        if (message) {
          if (queueType === 'telemetry') {
            this.sendTelemetry(message).catch(() => {
              messages.unshift(message);
            });
          } else if (queueType === 'task') {
            this.sendTask(message.task, message.priority).catch(() => {
              messages.unshift(message);
            });
          } else if (queueType === 'state') {
            this.updateDeviceState(message).catch(() => {
              messages.unshift(message);
            });
          }
        }
      }
    }
  }

  // Generate unique session identifier
  private generateSessionId(): string {
    return `sess_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  // Update last activity timestamp for session management
  private updateLastActivity(): void {
    this.lastActivity = Date.now();
  }

  // Setup internal listeners for subsystem events
  private setupInternalListeners(): void {
    this.heartbeat.onTimeout(() => {
      this.internalState = SynapseNetState.DEGRADED;
      this.emitInternalEvent(SynapseEvent.HEARTBEAT_TIMEOUT, { timestamp: Date.now() });
      if (this.config.autoReconnect) {
        this.connection.reconnect();
      }
    });
    this.heartbeat.onRestored(() => {
      this.emitInternalEvent(SynapseEvent.HEARTBEAT_RESTORED, { timestamp: Date.now() });
    });
    this.connection.onDisconnected(() => {
      this.internalState = SynapseNetState.DISCONNECTED;
      this.heartbeat.stop();
      this.emitInternalEvent(SynapseEvent.DISCONNECTED, { timestamp: Date.now() });
    });
    this.connection.onReconnecting((attempt) => {
      this.emitInternalEvent(SynapseEvent.RECONNECTING, { attempt, timestamp: Date.now() });
    });
  }

  // Start monitoring subsystem health
  private startHealthMonitoring(): void {
    setInterval(() => {
      this.checkComponentHealth();
    }, 60000);
  }

  // Check health of all components
  private checkComponentHealth(): void {
    const components = [
      { name: 'transport', status: this.transport.isHealthy() },
      { name: 'heartbeat', status: this.heartbeat.isActive() },
      { name: 'connection', status: this.connection.isConnected() }
    ];
    components.forEach(component => {
      this.componentStatus.set(component.name, component.status);
    });
  }

  // Get current SynapseNet state
  public getState(): SynapseNetState {
    return this.internalState;
  }

  // Get session identifier
  public getSessionId(): string | null {
    return this.sessionId;
  }

  // Graceful shutdown of all subsystems
  public async disconnect(): Promise<void> {
    this.heartbeat.stop();
    await this.connection.disconnect();
    this.internalState = SynapseNetState.DISCONNECTED;
    this.sessionId = null;
    this.emitInternalEvent(SynapseEvent.DISCONNECTED, { timestamp: Date.now() });
  }
}

// SynapseNet internal state enumeration
enum SynapseNetState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  DEGRADED = 'degraded',
  RECONNECTING = 'reconnecting'
}