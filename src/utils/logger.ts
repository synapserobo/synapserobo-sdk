/**
 * Advanced logging system for SynapseRobo SDK
 * Supports structured logging, telemetry, and multiple output formats
 */

import { LOG_LEVELS, DEFAULT_CONFIG } from '../constants';
import { LoggingConfig, SDKEvent } from './types';

export class Logger {
  private static instance: Logger;
  private config: LoggingConfig;
  private logBuffer: any[] = [];
  private readonly maxBufferSize = 1000;
  private telemetryEndpoint: string = 'https://telemetry.synapserobotics.xyz/v1/metrics';
  private isDevelopment: boolean = process.env.NODE_ENV === 'development';

  private constructor(config: LoggingConfig) {
    this.config = { ...DEFAULT_CONFIG.logging, ...config };
    this.setupUncaughtExceptionHandlers();
  }

  public static getInstance(config?: LoggingConfig): Logger {
    if (!Logger.instance) {
      Logger.instance = new Logger(config || DEFAULT_CONFIG.logging);
    }
    return Logger.instance;
  }

  public configure(newConfig: Partial<LoggingConfig>): void {
    this.config = { ...this.config, ...newConfig };
    this.info('Logger configuration updated', { newConfig });
  }

  private setupUncaughtExceptionHandlers(): void {
    process.on('uncaughtException', (error) => {
      this.error('Uncaught Exception', error, {
        process: 'node',
        pid: process.pid,
        stack: error.stack
      });
    });

    process.on('unhandledRejection', (reason, promise) => {
      this.error('Unhandled Promise Rejection', reason as Error, {
        promise: promise.toString(),
        reason: reason
      });
    });
  }

  private shouldLog(level: number): boolean {
    return level <= this.config.level;
  }

  private formatMessage(level: string, message: string, meta?: any): any {
    const baseEntry = {
      timestamp: new Date().toISOString(),
      level,
      message,
      pid: process.pid,
      hostname: require('os').hostname(),
      service: 'synapse-sdk',
      version: process.env.npm_package_version || '0.1.0',
      ...meta
    };

    if (this.config.format === 'json') {
      return baseEntry;
    } else {
      const time = new Date().toISOString();
      const metaStr = meta ? JSON.stringify(meta, this.safeStringify) : '';
      return `[${time}] ${level.toUpperCase()}: ${message} ${metaStr}`.trim();
    }
  }

  private safeStringify(key: string, value: any): any {
    // Handle circular references and large buffers
    if (value instanceof Buffer) {
      return `<Buffer ${value.length} bytes>`;
    }
    if (value instanceof Error) {
      return {
        name: value.name,
        message: value.message,
        stack: value.stack
      };
    }
    return value;
  }

  private writeToBuffer(entry: any): void {
    this.logBuffer.push(entry);
    if (this.logBuffer.length > this.maxBufferSize) {
      this.logBuffer.shift();
    }

    if (this.config.enableTelemetry) {
      this.sendTelemetry(entry);
    }
  }

  private async sendTelemetry(entry: any): Promise<void> {
    if (!this.config.enableTelemetry || this.isDevelopment) {
      return;
    }

    try {
      const telemetryData = {
        ...entry,
        sdk_version: '0.1.0',
        environment: process.env.NODE_ENV || 'production',
        timestamp: new Date().toISOString()
      };

      // In production, this would send to telemetry service
      // For now, we'll simulate the behavior
      if (process.env.NODE_ENV === 'production') {
        // Simulated telemetry sending - would be fetch/axios in real implementation
        setImmediate(() => {
          console.log(`[TELEMETRY] ${JSON.stringify(telemetryData)}`);
        });
      }
    } catch (error) {
      // Don't log telemetry errors to avoid infinite loops
      console.error('Telemetry error:', error);
    }
  }

  public error(message: string, error?: Error, meta?: any): void {
    if (!this.shouldLog(LOG_LEVELS.ERROR)) return;

    const entry = this.formatMessage('error', message, {
      ...meta,
      error: error ? {
        name: error.name,
        message: error.message,
        stack: this.isDevelopment ? error.stack : undefined
      } : undefined
    });

    console.error(this.config.format === 'json' ? JSON.stringify(entry) : entry);
    this.writeToBuffer(entry);
  }

  public warn(message: string, meta?: any): void {
    if (!this.shouldLog(LOG_LEVELS.WARN)) return;

    const entry = this.formatMessage('warn', message, meta);
    console.warn(this.config.format === 'json' ? JSON.stringify(entry) : entry);
    this.writeToBuffer(entry);
  }

  public info(message: string, meta?: any): void {
    if (!this.shouldLog(LOG_LEVELS.INFO)) return;

    const entry = this.formatMessage('info', message, meta);
    console.log(this.config.format === 'json' ? JSON.stringify(entry) : entry);
    this.writeToBuffer(entry);
  }

  public debug(message: string, meta?: any): void {
    if (!this.shouldLog(LOG_LEVELS.DEBUG)) return;

    const entry = this.formatMessage('debug', message, meta);
    console.debug(this.config.format === 'json' ? JSON.stringify(entry) : entry);
    this.writeToBuffer(entry);
  }

  public trace(message: string, meta?: any): void {
    if (!this.shouldLog(LOG_LEVELS.TRACE)) return;

    const entry = this.formatMessage('trace', message, meta);
    console.trace(this.config.format === 'json' ? JSON.stringify(entry) : entry);
    this.writeToBuffer(entry);
  }

  public metric(name: string, value: number, tags: Record<string, string> = {}): void {
    if (!this.config.enableTelemetry) return;

    const metricEntry = {
      type: 'metric',
      name,
      value,
      tags: {
        ...tags,
        service: 'synapse-sdk',
        environment: process.env.NODE_ENV || 'development'
      },
      timestamp: new Date().toISOString()
    };

    this.writeToBuffer(metricEntry);
    
    if (this.config.format !== 'json') {
      console.log(`[METRIC] ${name}=${value} ${JSON.stringify(tags)}`);
    }
  }

  public event(name: string, properties: Record<string, any> = {}): void {
    if (!this.config.enableTelemetry) return;

    const eventEntry = {
      type: 'event',
      name,
      properties: {
        ...properties,
        sdk_version: '0.1.0',
        environment: process.env.NODE_ENV || 'development'
      },
      timestamp: new Date().toISOString()
    };

    this.writeToBuffer(eventEntry);
    
    if (this.config.format !== 'json') {
      console.log(`[EVENT] ${name} ${JSON.stringify(properties)}`);
    }
  }

  public audit(action: string, user: string, resource: string, outcome: 'success' | 'failure', details?: any): void {
    const auditEntry = {
      type: 'audit',
      action,
      user,
      resource,
      outcome,
      timestamp: new Date().toISOString(),
      details,
      ip: '127.0.0.1', // Would be real IP in production
      user_agent: 'synapse-sdk'
    };

    this.writeToBuffer(auditEntry);
    
    if (this.config.format !== 'json') {
      console.log(`[AUDIT] ${user} ${action} ${resource} - ${outcome}`);
    }
  }

  public profile(operation: string, startTime: number, meta?: any): void {
    const duration = Date.now() - startTime;
    
    this.metric('operation_duration', duration, {
      operation,
      ...meta
    });

    if (duration > 1000) { // Log slow operations
      this.warn('Slow operation detected', {
        operation,
        duration,
        threshold: 1000,
        ...meta
      });
    }
  }

  public createRequestLogger(requestId: string, additionalContext?: any): RequestLogger {
    return new RequestLogger(this, requestId, additionalContext);
  }

  public getLogs(filter?: { 
    level?: number; 
    startTime?: Date; 
    endTime?: Date;
    type?: string;
  }): any[] {
    let filtered = this.logBuffer;

    if (filter?.level !== undefined) {
      filtered = filtered.filter(entry => {
        const levelMap: Record<string, number> = {
          error: LOG_LEVELS.ERROR,
          warn: LOG_LEVELS.WARN,
          info: LOG_LEVELS.INFO,
          debug: LOG_LEVELS.DEBUG,
          trace: LOG_LEVELS.TRACE
        };
        return levelMap[entry.level] <= filter.level!;
      });
    }

    if (filter?.startTime) {
      filtered = filtered.filter(entry => new Date(entry.timestamp) >= filter.startTime!);
    }

    if (filter?.endTime) {
      filtered = filtered.filter(entry => new Date(entry.timestamp) <= filter.endTime!);
    }

    if (filter?.type) {
      filtered = filtered.filter(entry => entry.type === filter.type);
    }

    return filtered;
  }

  public clearLogs(): void {
    this.logBuffer = [];
    this.info('Log buffer cleared');
  }

  public async flush(): Promise<void> {
    // In production, this would flush logs to persistent storage
    if (this.logBuffer.length > 0) {
      this.debug('Flushing logs to storage', { count: this.logBuffer.length });
      // Simulate async flush operation
      await new Promise(resolve => setTimeout(resolve, 100));
      this.logBuffer = [];
    }
  }

  public getStats(): LogStats {
    const levels = this.logBuffer.reduce((acc, entry) => {
      acc[entry.level] = (acc[entry.level] || 0) + 1;
      return acc;
    }, {} as Record<string, number>);

    return {
      total: this.logBuffer.length,
      levels,
      oldest: this.logBuffer[0]?.timestamp,
      newest: this.logBuffer[this.logBuffer.length - 1]?.timestamp,
      bufferSize: this.logBuffer.length
    };
  }
}

export class RequestLogger {
  constructor(
    private parent: Logger,
    private requestId: string,
    private context: Record<string, any> = {}
  ) {}

  public error(message: string, error?: Error, meta?: any): void {
    this.parent.error(message, error, { 
      requestId: this.requestId, 
      ...this.context, 
      ...meta 
    });
  }

  public warn(message: string, meta?: any): void {
    this.parent.warn(message, { 
      requestId: this.requestId, 
      ...this.context, 
      ...meta 
    });
  }

  public info(message: string, meta?: any): void {
    this.parent.info(message, { 
      requestId: this.requestId, 
      ...this.context, 
      ...meta 
    });
  }

  public debug(message: string, meta?: any): void {
    this.parent.debug(message, { 
      requestId: this.requestId, 
      ...this.context, 
      ...meta 
    });
  }

  public trace(message: string, meta?: any): void {
    this.parent.trace(message, { 
      requestId: this.requestId, 
      ...this.context, 
      ...meta 
    });
  }

  public metric(name: string, value: number, tags: Record<string, string> = {}): void {
    this.parent.metric(name, value, { 
      requestId: this.requestId, 
      ...this.context, 
      ...tags 
    });
  }

  public event(name: string, properties: Record<string, any> = {}): void {
    this.parent.event(name, { 
      requestId: this.requestId, 
      ...this.context, 
      ...properties 
    });
  }

  public profile(operation: string, startTime: number, meta?: any): void {
    this.parent.profile(operation, startTime, { 
      requestId: this.requestId, 
      ...this.context, 
      ...meta 
    });
  }
}

export class ChildLogger {
  constructor(
    private parent: Logger,
    private context: Record<string, any>
  ) {}

  public error(message: string, error?: Error, meta?: any): void {
    this.parent.error(message, error, { ...this.context, ...meta });
  }

  public warn(message: string, meta?: any): void {
    this.parent.warn(message, { ...this.context, ...meta });
  }

  public info(message: string, meta?: any): void {
    this.parent.info(message, { ...this.context, ...meta });
  }

  public debug(message: string, meta?: any): void {
    this.parent.debug(message, { ...this.context, ...meta });
  }

  public trace(message: string, meta?: any): void {
    this.parent.trace(message, { ...this.context, ...meta });
  }

  public metric(name: string, value: number, tags: Record<string, string> = {}): void {
    this.parent.metric(name, value, { ...this.context, ...tags });
  }

  public event(name: string, properties: Record<string, any> = {}): void {
    this.parent.event(name, { ...this.context, ...properties });
  }

  public audit(action: string, user: string, resource: string, outcome: 'success' | 'failure', details?: any): void {
    this.parent.audit(action, user, resource, outcome, { ...this.context, ...details });
  }

  public profile(operation: string, startTime: number, meta?: any): void {
    this.parent.profile(operation, startTime, { ...this.context, ...meta });
  }

  public createRequestLogger(requestId: string, additionalContext?: any): RequestLogger {
    return this.parent.createRequestLogger(requestId, { 
      ...this.context, 
      ...additionalContext 
    });
  }
}

// Interface for log statistics
interface LogStats {
  total: number;
  levels: Record<string, number>;
  oldest?: string;
  newest?: string;
  bufferSize: number;
}

// Export singleton instance
export const logger = Logger.getInstance();