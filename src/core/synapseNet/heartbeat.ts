import { EventEmitter } from 'events';

interface HeartbeatConfig {
  interval: number;
  timeout: number;
  maxMissedBeats?: number;
  enabled?: boolean;
}

interface HeartbeatStats {
  totalBeats: number;
  missedBeats: number;
  averageLatency: number;
  lastLatency: number;
  startedAt: number;
  lastBeatAt: number;
}

// Main HeartbeatManager class for session health monitoring
export class HeartbeatManager {
  private config: HeartbeatConfig;
  private intervalId: NodeJS.Timeout | null;
  private timeoutId: NodeJS.Timeout | null;
  private lastBeatTime: number | null;
  private isActive: boolean;
  private missedBeats: number;
  private eventEmitter: EventEmitter;
  private stats: HeartbeatStats;
  private checkCount: number;
  private recoveryThreshold: number;
  private degradationThreshold: number;
  private customBeatHandlers: Map<string, Function>;

  constructor() {
    this.config = {
      interval: 30000,
      timeout: 90000,
      maxMissedBeats: 3,
      enabled: true
    };
    this.intervalId = null;
    this.timeoutId = null;
    this.lastBeatTime = null;
    this.isActive = false;
    this.missedBeats = 0;
    this.eventEmitter = new EventEmitter();
    this.stats = {
      totalBeats: 0,
      missedBeats: 0,
      averageLatency: 0,
      lastLatency: 0,
      startedAt: 0,
      lastBeatAt: 0
    };
    this.checkCount = 0;
    this.recoveryThreshold = 2;
    this.degradationThreshold = 2;
    this.customBeatHandlers = new Map();
  }

  // Initialize heartbeat manager with configuration
  public initialize(config?: Partial<HeartbeatConfig>): void {
    this.config = { ...this.config, ...config };
    this.stats.startedAt = Date.now();
    this.eventEmitter.emit('initialized', { config: this.config, timestamp: Date.now() });
  }

  // Start heartbeat monitoring with specified interval
  public start(interval?: number): void {
    if (interval) {
      this.config.interval = interval;
    }
    
    if (!this.config.enabled) {
      return;
    }
    
    if (this.intervalId) {
      clearInterval(this.intervalId);
    }
    
    this.isActive = true;
    this.lastBeatTime = Date.now();
    this.missedBeats = 0;
    
    this.intervalId = setInterval(() => {
      this.sendBeat();
    }, this.config.interval);
    
    this.resetTimeout();
    this.eventEmitter.emit('started', { interval: this.config.interval, timestamp: Date.now() });
  }

  public stop(): void {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }
    
    if (this.timeoutId) {
      clearTimeout(this.timeoutId);
      this.timeoutId = null;
    }
    
    this.isActive = false;
    this.eventEmitter.emit('stopped', { timestamp: Date.now() });
  }

  // Send heartbeat signal and update statistics
  private sendBeat(): void {
    const beatTime = Date.now();
    const beatId = this.generateBeatId();
    
    this.checkCount++;
    this.executeCustomBeats();
    
    const beatEvent = {
      id: beatId,
      timestamp: beatTime,
      checkCount: this.checkCount,
      missedBeats: this.missedBeats,
      stats: { ...this.stats }
    };
    
    this.eventEmitter.emit('beat_sent', beatEvent);
    this.lastBeatTime = beatTime;
    this.stats.totalBeats++;
    this.stats.lastBeatAt = beatTime;
    this.resetTimeout();
    
    this.updateStats();
  }

  // Register custom beat handler for additional health checks
  public registerCustomBeat(name: string, handler: Function, intervalMultiplier: number = 1): void {
    this.customBeatHandlers.set(name, handler);
    
    if (intervalMultiplier > 1) {
      setInterval(() => {
        if (this.isActive) {
          handler();
        }
      }, this.config.interval * intervalMultiplier);
    }
  }

  // Execute all registered custom beat handlers
  private executeCustomBeats(): void {
    for (const [name, handler] of this.customBeatHandlers.entries()) {
      try {
        handler();
      } catch (error) {
        this.eventEmitter.emit('custom_beat_error', { name, error, timestamp: Date.now() });
      }
    }
  }

  private resetTimeout(): void {
    if (this.timeoutId) {
      clearTimeout(this.timeoutId);
    }
    
    this.timeoutId = setTimeout(() => {
      this.handleTimeout();
    }, this.config.timeout);
  }

  private handleTimeout(): void {
    this.missedBeats++;
    this.stats.missedBeats++;
    
    const timeoutEvent = {
      missedBeats: this.missedBeats,
      maxAllowed: this.config.maxMissedBeats,
      lastBeatTime: this.lastBeatTime,
      timestamp: Date.now(),
      checkCount: this.checkCount
    };
    
    this.eventEmitter.emit('beat_missed', timeoutEvent);
    
    if (this.missedBeats >= this.degradationThreshold) {
      this.eventEmitter.emit('degradation', {
        level: 'warning',
        missedBeats: this.missedBeats,
        timestamp: Date.now()
      });
    }
    
    if (this.missedBeats >= this.config.maxMissedBeats!) {
      this.handleCompleteTimeout();
    }
  }

  private handleCompleteTimeout(): void {
    const timeoutEvent = {
      reason: 'max_missed_beats_exceeded',
      missedBeats: this.missedBeats,
      lastBeatTime: this.lastBeatTime,
      timestamp: Date.now(),
      stats: { ...this.stats }
    };
    
    this.eventEmitter.emit('timeout', timeoutEvent);
    this.isActive = false;
    this.stop();
  }

  // Receive heartbeat acknowledgment from remote
  public receiveAck(latency?: number): void {
    if (!this.isActive) {
      return;
    }
    
    const ackTime = Date.now();
    const actualLatency = latency || (this.lastBeatTime ? ackTime - this.lastBeatTime : 0);
    
    this.missedBeats = Math.max(0, this.missedBeats - 1);
    this.stats.lastLatency = actualLatency;
    this.updateAverageLatency(actualLatency);
    
    if (this.timeoutId) {
      clearTimeout(this.timeoutId);
      this.resetTimeout();
    }
    
    const recoveryEvent = {
      latency: actualLatency,
      missedBeats: this.missedBeats,
      timestamp: ackTime,
      averageLatency: this.stats.averageLatency
    };
    
    this.eventEmitter.emit('ack_received', recoveryEvent);
    
    if (this.missedBeats === 0 && !this.isActive) {
      this.isActive = true;
      this.eventEmitter.emit('recovered', {
        downtime: ackTime - (this.lastBeatTime || ackTime),
        timestamp: ackTime
      });
    }
  }

  // Update average latency calculation
  private updateAverageLatency(newLatency: number): void {
    if (this.stats.totalBeats === 0) {
      this.stats.averageLatency = newLatency;
    } else {
      this.stats.averageLatency = 
        (this.stats.averageLatency * (this.stats.totalBeats - 1) + newLatency) / this.stats.totalBeats;
    }
  }

  private updateStats(): void {
    const now = Date.now();
    const uptime = now - this.stats.startedAt;
    const availability = this.stats.totalBeats > 0 ? 
      ((this.stats.totalBeats - this.stats.missedBeats) / this.stats.totalBeats) * 100 : 100;
    
    this.stats = {
      ...this.stats,
      averageLatency: this.stats.averageLatency,
      lastLatency: this.stats.lastLatency
    };
    
    if (this.checkCount % 10 === 0) {
      this.eventEmitter.emit('stats_updated', {
        stats: { ...this.stats, uptime, availability },
        timestamp: now
      });
    }
  }

  // Generate unique heartbeat identifier
  private generateBeatId(): string {
    return `beat_${Date.now()}_${Math.random().toString(36).substr(2, 6)}`;
  }

  public isActiveStatus(): boolean {
    return this.isActive;
  }

  public getStats(): HeartbeatStats {
    return { ...this.stats };
  }

  public getConfig(): HeartbeatConfig {
    return { ...this.config };
  }

  // Update configuration dynamically
  public updateConfig(newConfig: Partial<HeartbeatConfig>): void {
    const wasActive = this.isActive;
    
    if (wasActive) {
      this.stop();
    }
    
    this.config = { ...this.config, ...newConfig };
    
    if (wasActive) {
      this.start();
    }
    
    this.eventEmitter.emit('config_updated', { config: this.config, timestamp: Date.now() });
  }

  public onTimeout(handler: Function): void {
    this.eventEmitter.on('timeout', handler);
  }

  public onRestored(handler: Function): void {
    this.eventEmitter.on('recovered', handler);
  }

  public onBeatSent(handler: Function): void {
    this.eventEmitter.on('beat_sent', handler);
  }

  public onAckReceived(handler: Function): void {
    this.eventEmitter.on('ack_received', handler);
  }

  public performHealthCheck(): { healthy: boolean; details: any } {
    const now = Date.now();
    const timeSinceLastBeat = this.lastBeatTime ? now - this.lastBeatTime : Infinity;
    const isTimedOut = timeSinceLastBeat > this.config.timeout;
    
    const healthDetails = {
      isActive: this.isActive,
      timeSinceLastBeat,
      missedBeats: this.missedBeats,
      maxMissedBeats: this.config.maxMissedBeats,
      isTimedOut,
      stats: { ...this.stats },
      timestamp: now
    };
    
    const healthy = this.isActive && !isTimedOut && this.missedBeats < this.config.maxMissedBeats!;
    
    this.eventEmitter.emit('health_check', { healthy, details: healthDetails });
    
    return { healthy, details: healthDetails };
  }

  // Reset heartbeat state
  public reset(): void {
    this.stop();
    this.missedBeats = 0;
    this.checkCount = 0;
    this.lastBeatTime = null;
    this.stats = {
      totalBeats: 0,
      missedBeats: 0,
      averageLatency: 0,
      lastLatency: 0,
      startedAt: Date.now(),
      lastBeatAt: 0
    };
    this.eventEmitter.emit('reset', { timestamp: Date.now() });
  }

  // Simulate heartbeat for testing or manual triggering
  public simulateBeat(latency?: number): void {
    this.sendBeat();
    if (latency !== undefined) {
      setTimeout(() => {
        this.receiveAck(latency);
      }, latency);
    }
  }

  public getTimeSinceLastBeat(): number | null {
    if (!this.lastBeatTime) return null;
    return Date.now() - this.lastBeatTime;
  }

  public isDegraded(): boolean {
    return this.missedBeats >= this.degradationThreshold && this.missedBeats < this.config.maxMissedBeats!;
  }

  // Set custom thresholds for degradation and recovery
  public setThresholds(degradation: number, recovery: number): void {
    this.degradationThreshold = degradation;
    this.recoveryThreshold = recovery;
    this.eventEmitter.emit('thresholds_updated', { degradation, recovery, timestamp: Date.now() });
  }
}