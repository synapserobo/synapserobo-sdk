import { TargetNode } from './registry';
import { EventEmitter } from 'events';

export interface Task {
  id: string;
  type: string;
  payload: Record<string, any>;
  priority: number; // 1-10, 1 is highest
  createdAt: number;
  timeoutMs: number;
  metadata: Record<string, any>;
  requiredCapabilities: Record<string, any>;
}

export type TaskStatus = 'pending' | 'assigned' | 'running' | 'completed' | 'failed' | 'cancelled';

export interface QueueEntry {
  task: Task;
  status: TaskStatus;
  assignedTo?: string; // Target ID
  assignedAt?: number;
  startedAt?: number;
  completedAt?: number;
  retryCount: number;
  maxRetries: number;
  lastError?: string;
  sequenceNumber: number;
}

export interface QueueStats {
  totalTasks: number;
  pendingTasks: number;
  assignedTasks: number;
  runningTasks: number;
  completedTasks: number;
  failedTasks: number;
  averageWaitTime: number;
  maxWaitTime: number;
}

// Queue configuration
interface QueueConfig {
  maxQueueSize: number;
  defaultMaxRetries: number;
  priorityBoostInterval: number; // Priority increases over time
  stallTimeoutMs: number; // Task considered stalled if not started
  cleanupCompletedAfterMs: number; // Remove completed tasks after
  deduplicationWindowMs: number; // Window for duplicate detection
}

// Default configuration
const DEFAULT_CONFIG: QueueConfig = {
  maxQueueSize: 10000,
  defaultMaxRetries: 3,
  priorityBoostInterval: 30000, // 30 seconds
  stallTimeoutMs: 60000, // 1 minute
  cleanupCompletedAfterMs: 3600000, // 1 hour
  deduplicationWindowMs: 300000 // 5 minutes
};

function taskHash(task: Task): string {
  const str = `${task.type}:${JSON.stringify(task.payload)}:${task.priority}`;
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    const char = str.charCodeAt(i);
    hash = ((hash << 5) - hash) + char;
    hash = hash & hash; 
  }
  return Math.abs(hash).toString(16);
}

// Main task queue class
export class TaskQueue extends EventEmitter {
  private queue: QueueEntry[];
  private sequenceCounter: number;
  private taskHashes: Map<string, { entryId: string; timestamp: number }>;
  private config: QueueConfig;
  private stats: {
    tasksCompleted: number;
    tasksFailed: number;
    totalWaitTime: number;
    maxWaitTime: number;
  };
  
  constructor(config: Partial<QueueConfig> = {}) {
    super();
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.queue = [];
    this.sequenceCounter = 0;
    this.taskHashes = new Map();
    this.stats = {
      tasksCompleted: 0,
      tasksFailed: 0,
      totalWaitTime: 0,
      maxWaitTime: 0
    };
    
    this.startMaintenanceLoop();
  }
  
  // Start maintenance loop for cleanup and priority adjustments
  private startMaintenanceLoop(): void {
    setInterval(() => {
      this.adjustPriorities();
      this.cleanupOldEntries();
      this.cleanupTaskHashes();
    }, 30000); 
  }
  
  private adjustPriorities(): void {
    const now = Date.now();
    const boostInterval = this.config.priorityBoostInterval;
    
    for (const entry of this.queue) {
      if (entry.status === 'pending') {
        const waitTime = now - entry.task.createdAt;
        const boostLevel = Math.floor(waitTime / boostInterval);
        
        if (boostLevel > 0) {
          // Increase priority based on wait time (capped at 1)
          const newPriority = Math.max(1, entry.task.priority - boostLevel);
          if (newPriority !== entry.task.priority) {
            entry.task.priority = newPriority;
            this.emit('priorityAdjusted', entry.task.id, newPriority);
          }
        }
      }
    }
    this.sortQueue();
  }
  
  private cleanupOldEntries(): void {
    const now = Date.now();
    const cleanupAge = this.config.cleanupCompletedAfterMs;
    
    // Keep only recent entries and those not completed/failed
    this.queue = this.queue.filter(entry => {
      if (entry.completedAt && (now - entry.completedAt) > cleanupAge) {
        this.emit('entryCleanedUp', entry);
        return false;
      }
      if (entry.status === 'failed' && entry.completedAt && 
          (now - entry.completedAt) > cleanupAge) {
        this.emit('entryCleanedUp', entry);
        return false;
      }
      return true;
    });
  }
  
  private cleanupTaskHashes(): void {
    const now = Date.now();
    const dedupWindow = this.config.deduplicationWindowMs;
    
    for (const [hash, data] of this.taskHashes.entries()) {
      if (now - data.timestamp > dedupWindow) {
        this.taskHashes.delete(hash);
      }
    }
  }
  
  // Sort queue by priority and creation time
  private sortQueue(): void {
    this.queue.sort((a, b) => {
      // First by priority (lower number = higher priority)
      if (a.task.priority !== b.task.priority) {
        return a.task.priority - b.task.priority;
      }
      // Then by creation time (older first)
      return a.task.createdAt - b.task.createdAt;
    });
  }
  
  // Enqueue a new task
  public enqueueTask(task: Task, assignedTo?: string): QueueEntry {
    // Check for duplicates
    const hash = taskHash(task);
    const existingHash = this.taskHashes.get(hash);
    
    if (existingHash && (Date.now() - existingHash.timestamp) < this.config.deduplicationWindowMs) {
      const existingEntry = this.queue.find(e => e.task.id === existingHash.entryId);
      if (existingEntry) {
        this.emit('duplicateTaskDetected', task, existingEntry);
        return existingEntry;
      }
    }
    
    // Check queue size limit
    if (this.queue.length >= this.config.maxQueueSize) {
      throw new Error(`Queue full: ${this.queue.length}/${this.config.maxQueueSize}`);
    }
    
    // Create queue entry
    const entry: QueueEntry = {
      task,
      status: assignedTo ? 'assigned' : 'pending',
      assignedTo,
      assignedAt: assignedTo ? Date.now() : undefined,
      retryCount: 0,
      maxRetries: this.config.defaultMaxRetries,
      sequenceNumber: this.sequenceCounter++
    };
    
    this.queue.push(entry);
    this.taskHashes.set(hash, { entryId: task.id, timestamp: Date.now() });
    
    // Sort queue after adding new entry
    this.sortQueue();
    
    this.emit('taskEnqueued', entry);
    return entry;
  }
  
  // Dequeue the highest priority task
  public dequeueTask(): QueueEntry | null {
    // Find first pending task
    const pendingIndex = this.queue.findIndex(entry => entry.status === 'pending');
    
    if (pendingIndex === -1) {
      return null;
    }
    
    const entry = this.queue[pendingIndex];
    entry.status = 'assigned';
    entry.assignedAt = Date.now();
    
    this.emit('taskDequeued', entry);
    return entry;
  }
  
  // Find task by ID
  public findTask(taskId: string): QueueEntry | undefined {
    return this.queue.find(entry => entry.task.id === taskId);
  }
  
  // Update task status
  public updateTaskStatus(taskId: string, status: TaskStatus, error?: string): boolean {
    const entry = this.findTask(taskId);
    if (!entry) return false;
    
    const oldStatus = entry.status;
    entry.status = status;
    
    if (status === 'running' && !entry.startedAt) {
      entry.startedAt = Date.now();
    }
    
    if (status === 'completed' || status === 'failed' || status === 'cancelled') {
      entry.completedAt = Date.now();
      
      // Update statistics
      if (status === 'completed') {
        this.stats.tasksCompleted++;
        if (entry.startedAt && entry.assignedAt) {
          const waitTime = entry.startedAt - entry.assignedAt;
          this.stats.totalWaitTime += waitTime;
          this.stats.maxWaitTime = Math.max(this.stats.maxWaitTime, waitTime);
        }
      } else if (status === 'failed') {
        this.stats.tasksFailed++;
      }
    }
    
    if (error) {
      entry.lastError = error;
    }
    
    this.emit('taskStatusChanged', taskId, oldStatus, status, error);
    return true;
  }
  
  // Assign task to target
  public assignTask(taskId: string, targetId: string): boolean {
    const entry = this.findTask(taskId);
    if (!entry || entry.status !== 'pending') return false;
    
    entry.status = 'assigned';
    entry.assignedTo = targetId;
    entry.assignedAt = Date.now();
    
    this.emit('taskAssigned', entry, targetId);
    return true;
  }
  
  // Retry a failed or stalled task
  public retryTask(taskId: string): boolean {
    const entry = this.findTask(taskId);
    if (!entry) return false;
    
    if (entry.retryCount >= entry.maxRetries) {
      this.updateTaskStatus(taskId, 'failed', 'Max retries exceeded');
      return false;
    }
    
    entry.retryCount++;
    entry.status = 'pending';
    entry.assignedTo = undefined;
    entry.assignedAt = undefined;
    entry.startedAt = undefined;
    entry.lastError = undefined;
    
    // Increase priority for retried tasks
    entry.task.priority = Math.max(1, entry.task.priority - 1);
    
    // Re-sort queue
    this.sortQueue();
    
    this.emit('taskRetried', entry);
    return true;
  }
  
  // Get stalled tasks (assigned but not started)
  public getStalledTasks(currentTime: number): QueueEntry[] {
    const stallTimeout = this.config.stallTimeoutMs;
    return this.queue.filter(entry => {
      if (entry.status !== 'assigned' || !entry.assignedAt) return false;
      return (currentTime - entry.assignedAt) > stallTimeout;
    });
  }
  
  // Cancel a task
  public cancelTask(taskId: string, reason?: string): boolean {
    const entry = this.findTask(taskId);
    if (!entry) return false;
    
    if (entry.status === 'completed' || entry.status === 'failed') {
      return false; // Cannot cancel completed/failed tasks
    }
    
    this.updateTaskStatus(taskId, 'cancelled', reason);
    return true;
  }
  
  // Get queue statistics
  public getStats(): QueueStats {
    const now = Date.now();
    let pendingCount = 0;
    let assignedCount = 0;
    let runningCount = 0;
    let completedCount = 0;
    let failedCount = 0;
    let totalWaitTime = 0;
    let waitCount = 0;
    
    for (const entry of this.queue) {
      switch (entry.status) {
        case 'pending': pendingCount++; break;
        case 'assigned': assignedCount++; break;
        case 'running': runningCount++; break;
        case 'completed': completedCount++; break;
        case 'failed': failedCount++; break;
      }
      
      if (entry.startedAt && entry.assignedAt) {
        totalWaitTime += (entry.startedAt - entry.assignedAt);
        waitCount++;
      }
    }
    
    const averageWaitTime = waitCount > 0 ? totalWaitTime / waitCount : 0;
    
    return {
      totalTasks: this.queue.length,
      pendingTasks: pendingCount,
      assignedTasks: assignedCount,
      runningTasks: runningCount,
      completedTasks: completedCount,
      failedTasks: failedCount,
      averageWaitTime,
      maxWaitTime: this.stats.maxWaitTime
    };
  }
  
  // Get current queue length
  public getQueueLength(): number {
    return this.queue.length;
  }
  
  // Get tasks by status
  public getTasksByStatus(status: TaskStatus): QueueEntry[] {
    return this.queue.filter(entry => entry.status === status);
  }
  
  // Get tasks assigned to specific target
  public getTasksByTarget(targetId: string): QueueEntry[] {
    return this.queue.filter(entry => entry.assignedTo === targetId);
  }
  
  // Clear all entries (for testing/reset)
  public clear(): void {
    this.queue = [];
    this.taskHashes.clear();
    this.sequenceCounter = 0;
    this.stats = {
      tasksCompleted: 0,
      tasksFailed: 0,
      totalWaitTime: 0,
      maxWaitTime: 0
    };
  }
  
  // Remove completed tasks from queue
  public purgeCompleted(): number {
    const beforeCount = this.queue.length;
    this.queue = this.queue.filter(entry => 
      entry.status !== 'completed' && entry.status !== 'failed' && entry.status !== 'cancelled'
    );
    return beforeCount - this.queue.length;
  }
  
  // Check for duplicate task
  public isDuplicate(task: Task): boolean {
    const hash = taskHash(task);
    const existingHash = this.taskHashes.get(hash);
    
    if (!existingHash) return false;
    
    // Check if the original task still exists in queue
    const existingEntry = this.queue.find(e => e.task.id === existingHash.entryId);
    return !!existingEntry && (Date.now() - existingHash.timestamp) < this.config.deduplicationWindowMs;
  }
}