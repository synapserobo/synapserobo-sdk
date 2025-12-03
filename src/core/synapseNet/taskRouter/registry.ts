import { EventEmitter } from 'events';

// Target capabilities interfaces
export interface RobotCapabilities {
  canNavigate: boolean;
  canManipulate: boolean;
  canSense: boolean;
  payloadCapacityKg: number;
  maxReachMm: number;
  supportedInterfaces: string[];
  firmwareVersion: string;
  batteryPowered: boolean;
}

export interface SimulationCapabilities {
  physicsEngine: string;
  maxSceneComplexity: number;
  simulationSpeed: number;
  supportedSensors: string[];
  renderQuality: 'low' | 'medium' | 'high';
  canSimulateRobots: number; // Max number of robots
}

// Base target node interface
export interface TargetNode {
  id: string;
  type: 'robot' | 'simulation';
  capabilities: RobotCapabilities | SimulationCapabilities;
  isOnline: boolean;
  lastSeen: number;
  currentLoad: number; // 0-1 scale
  maxLoad: number;     // Maximum load before considered busy
  isFallback: boolean;
  lastAssignedTask: number;
  tags: string[];
  location?: string; 
}

// Required capabilities for a task
export interface RequiredCapabilities {
  robot?: Partial<RobotCapabilities>;
  simulation?: Partial<SimulationCapabilities>;
  mustBeRealRobot?: boolean;
  minPayloadCapacityKg?: number;
  requiredInterfaces?: string[];
  locationPreference?: string;
}

// Registry statistics
export interface RegistryStats {
  totalTargets: number;
  onlineTargets: number;
  robotsOnline: number;
  simulationNodesOnline: number;
  averageLoad: number;
}

// Registry configuration
interface RegistryConfig {
  offlineTimeoutMs: number;
  maxLoadThreshold: number;
  selectionAlgorithm: 'deterministic' | 'roundRobin' | 'leastLoaded';
}

// Default configuration
const DEFAULT_CONFIG: RegistryConfig = {
  offlineTimeoutMs: 30000, // 30 seconds
  maxLoadThreshold: 0.85,  // 85% load considered busy
  selectionAlgorithm: 'deterministic'
};

// Deterministic selection seed for consistent routing
const SELECTION_SEED = 0x12345678;

// Main registry class
export class Registry extends EventEmitter {
  private targets: Map<string, TargetNode>;
  private config: RegistryConfig;
  private selectionIndex: Map<string, number>; // For round-robin selection
  private lastSelectionTime: number;
  
  constructor(config: Partial<RegistryConfig> = {}) {
    super();
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.targets = new Map();
    this.selectionIndex = new Map();
    this.lastSelectionTime = Date.now();
    this.startHealthMonitoring();
  }
  
  // Start health monitoring interval
  private startHealthMonitoring(): void {
    setInterval(() => {
      this.checkTargetHealth();
    }, 10000); // Check every 10 seconds
  }
  
  // Check target health and mark offline if stale
  private checkTargetHealth(): void {
    const now = Date.now();
    const offlineTimeout = this.config.offlineTimeoutMs;
    
    for (const target of this.targets.values()) {
      if (target.isOnline && (now - target.lastSeen) > offlineTimeout) {
        target.isOnline = false;
        this.emit('targetWentOffline', target.id, now - target.lastSeen);
      }
    }
  }
  
  // Register a robot target
  public registerRobot(
    id: string,
    capabilities: RobotCapabilities,
    load: number = 0
  ): boolean {
    if (this.targets.has(id)) {
      return false; // Already registered
    }
    
    const target: TargetNode = {
      id,
      type: 'robot',
      capabilities,
      isOnline: true,
      lastSeen: Date.now(),
      currentLoad: Math.min(load, 1),
      maxLoad: this.config.maxLoadThreshold,
      isFallback: false,
      lastAssignedTask: 0,
      tags: ['robot', `fw:${capabilities.firmwareVersion}`],
      location: undefined
    };
    
    this.targets.set(id, target);
    this.emit('targetRegistered', target);
    return true;
  }
  
  // Register a simulation node
  public registerSimulationNode(
    id: string,
    capabilities: SimulationCapabilities,
    load: number = 0
  ): boolean {
    if (this.targets.has(id)) {
      return false;
    }
    
    const target: TargetNode = {
      id,
      type: 'simulation',
      capabilities,
      isOnline: true,
      lastSeen: Date.now(),
      currentLoad: Math.min(load, 1),
      maxLoad: this.config.maxLoadThreshold,
      isFallback: false,
      lastAssignedTask: 0,
      tags: ['simulation', `physics:${capabilities.physicsEngine}`],
      location: undefined
    };
    
    this.targets.set(id, target);
    this.emit('targetRegistered', target);
    return true;
  }
  
  // Unregister a target
  public unregisterTarget(id: string): boolean {
    const target = this.targets.get(id);
    if (!target) return false;
    
    this.targets.delete(id);
    this.selectionIndex.delete(id);
    this.emit('targetUnregistered', target);
    return true;
  }
  
  // Update target status
  public updateTargetStatus(
    id: string,
    isOnline: boolean,
    load?: number,
    location?: string
  ): boolean {
    const target = this.targets.get(id);
    if (!target) return false;
    
    target.isOnline = isOnline;
    target.lastSeen = Date.now();
    
    if (load !== undefined) {
      target.currentLoad = Math.min(load, 1);
    }
    
    if (location !== undefined) {
      target.location = location;
    }
    
    this.emit('targetStatusUpdated', target);
    return true;
  }
  
  // Select available target based on requirements
  public selectAvailableTarget(
    requiredCaps?: RequiredCapabilities,
    priority: number = 5
  ): TargetNode | null {
    // Get all online targets
    const onlineTargets = Array.from(this.targets.values())
      .filter(target => target.isOnline && target.currentLoad < target.maxLoad);
    
    if (onlineTargets.length === 0) {
      return this.selectFallbackTarget();
    }
    
    // Filter by required capabilities
    const filteredTargets = onlineTargets.filter(target => 
      this.matchesCapabilities(target, requiredCaps)
    );
    
    if (filteredTargets.length === 0) {
      // No targets match exact capabilities, try relaxed matching
      return this.selectRelaxedTarget(onlineTargets, requiredCaps);
    }
    
    // Apply deterministic selection algorithm
    return this.deterministicSelect(filteredTargets, priority);
  }
  
  // Check if target matches required capabilities
  private matchesCapabilities(
    target: TargetNode,
    requiredCaps?: RequiredCapabilities
  ): boolean {
    if (!requiredCaps) return true;
    
    // Check if must be real robot
    if (requiredCaps.mustBeRealRobot && target.type !== 'robot') {
      return false;
    }
    
    // Check robot-specific capabilities
    if (target.type === 'robot' && requiredCaps.robot) {
      const robotCaps = target.capabilities as RobotCapabilities;
      const requiredRobotCaps = requiredCaps.robot;
      
      if (requiredRobotCaps.canNavigate !== undefined && 
          requiredRobotCaps.canNavigate !== robotCaps.canNavigate) {
        return false;
      }
      
      if (requiredRobotCaps.canManipulate !== undefined && 
          requiredRobotCaps.canManipulate !== robotCaps.canManipulate) {
        return false;
      }
      
      if (requiredRobotCaps.payloadCapacityKg !== undefined && 
          robotCaps.payloadCapacityKg < requiredRobotCaps.payloadCapacityKg) {
        return false;
      }
      
      if (requiredCaps.minPayloadCapacityKg !== undefined && 
          robotCaps.payloadCapacityKg < requiredCaps.minPayloadCapacityKg) {
        return false;
      }
      
      if (requiredRobotCaps.supportedInterfaces) {
        const hasAllInterfaces = requiredRobotCaps.supportedInterfaces.every(
          iface => robotCaps.supportedInterfaces.includes(iface)
        );
        if (!hasAllInterfaces) return false;
      }
    }
    
    // Check simulation-specific capabilities
    if (target.type === 'simulation' && requiredCaps.simulation) {
      const simCaps = target.capabilities as SimulationCapabilities;
      const requiredSimCaps = requiredCaps.simulation;
      
      if (requiredSimCaps.canSimulateRobots !== undefined && 
          simCaps.canSimulateRobots < requiredSimCaps.canSimulateRobots) {
        return false;
      }
      
      if (requiredSimCaps.supportedSensors) {
        const hasAllSensors = requiredSimCaps.supportedSensors.every(
          sensor => simCaps.supportedSensors.includes(sensor)
        );
        if (!hasAllSensors) return false;
      }
    }
    
    // Check location preference
    if (requiredCaps.locationPreference && target.location) {
      if (!target.location.includes(requiredCaps.locationPreference)) {
        return false;
      }
    }
    
    // Check required interfaces
    if (requiredCaps.requiredInterfaces && target.type === 'robot') {
      const robotCaps = target.capabilities as RobotCapabilities;
      const hasAllInterfaces = requiredCaps.requiredInterfaces.every(
        iface => robotCaps.supportedInterfaces.includes(iface)
      );
      if (!hasAllInterfaces) return false;
    }
    
    return true;
  }
  
  // Select target with relaxed capability matching
  private selectRelaxedTarget(
    targets: TargetNode[],
    requiredCaps?: RequiredCapabilities
  ): TargetNode | null {
    if (!requiredCaps) {
      return this.deterministicSelect(targets, 5);
    }
    
    // Score each target based on how well it matches relaxed requirements
    const scoredTargets = targets.map(target => {
      let score = 0;
      
      // Basic type matching
      if (requiredCaps.mustBeRealRobot && target.type === 'robot') {
        score += 100;
      } else if (!requiredCaps.mustBeRealRobot) {
        score += 50;
      }
      
      // Load score (prefer less loaded)
      score += (1 - target.currentLoad) * 30;
      
      // Capability matching for robots
      if (target.type === 'robot' && requiredCaps.robot) {
        const robotCaps = target.capabilities as RobotCapabilities;
        const requiredRobotCaps = requiredCaps.robot;
        
        if (requiredRobotCaps.payloadCapacityKg && 
            robotCaps.payloadCapacityKg >= requiredRobotCaps.payloadCapacityKg) {
          score += 20;
        }
        
        if (requiredRobotCaps.supportedInterfaces) {
          const matchingInterfaces = requiredRobotCaps.supportedInterfaces.filter(
            iface => robotCaps.supportedInterfaces.includes(iface)
          ).length;
          score += matchingInterfaces * 5;
        }
      }
      
      return { target, score };
    });
    
    // Sort by score descending
    scoredTargets.sort((a, b) => b.score - a.score);
    
    if (scoredTargets.length > 0 && scoredTargets[0].score > 0) {
      return scoredTargets[0].target;
    }
    
    return null;
  }
  
  // Deterministic target selection
  private deterministicSelect(targets: TargetNode[], priority: number): TargetNode | null {
    if (targets.length === 0) return null;
    if (targets.length === 1) return targets[0];
    
    // Create deterministic hash based on targets and priority
    const targetIds = targets.map(t => t.id).sort();
    const hashInput = targetIds.join('|') + '|' + priority + '|' + SELECTION_SEED;
    
    // Simple deterministic hash function
    let hash = 0;
    for (let i = 0; i < hashInput.length; i++) {
      hash = ((hash << 5) - hash) + hashInput.charCodeAt(i);
      hash |= 0; // Convert to 32-bit integer
    }
    
    // Use absolute value and modulo to select index
    const index = Math.abs(hash) % targets.length;
    const selectedTarget = targets[index];
    
    // Update last assigned time for load balancing
    selectedTarget.lastAssignedTask = Date.now();
    
    return selectedTarget;
  }
  
  // Select fallback target (offline-capable or simulation)
  private selectFallbackTarget(): TargetNode | null {
    // First, try to find any online target even if loaded
    const anyOnline = Array.from(this.targets.values())
      .filter(target => target.isOnline);
    
    if (anyOnline.length > 0) {
      // Return the least loaded online target
      return anyOnline.reduce((prev, current) => 
        prev.currentLoad < current.currentLoad ? prev : current
      );
    }
    
    // If no online targets, check for simulation nodes that can run offline
    const simulationNodes = Array.from(this.targets.values())
      .filter(target => target.type === 'simulation');
    
    if (simulationNodes.length > 0) {
      const selected = simulationNodes[0];
      selected.isFallback = true; // Mark as fallback selection
      return selected;
    }
    
    return null; // No targets available
  }
  
  // Get registry statistics
  public getStats(): RegistryStats {
    const targets = Array.from(this.targets.values());
    const onlineTargets = targets.filter(t => t.isOnline);
    const robotsOnline = onlineTargets.filter(t => t.type === 'robot').length;
    const simNodesOnline = onlineTargets.filter(t => t.type === 'simulation').length;
    
    const totalLoad = onlineTargets.reduce((sum, target) => sum + target.currentLoad, 0);
    const averageLoad = onlineTargets.length > 0 ? totalLoad / onlineTargets.length : 0;
    
    return {
      totalTargets: targets.length,
      onlineTargets: onlineTargets.length,
      robotsOnline,
      simulationNodesOnline: simNodesOnline,
      averageLoad
    };
  }
  
  // Get all targets
  public getAllTargets(): TargetNode[] {
    return Array.from(this.targets.values());
  }
  
  // Get target by ID
  public getTarget(id: string): TargetNode | undefined {
    return this.targets.get(id);
  }
  
  // Clear all targets (for testing/reset)
  public clear(): void {
    this.targets.clear();
    this.selectionIndex.clear();
  }
  
  // Update target capabilities
  public updateTargetCapabilities(
    id: string,
    capabilities: Partial<RobotCapabilities | SimulationCapabilities>
  ): boolean {
    const target = this.targets.get(id);
    if (!target) return false;
    
    Object.assign(target.capabilities, capabilities);
    target.lastSeen = Date.now();
    return true;
  }
  
  // Add tag to target
  public addTargetTag(id: string, tag: string): boolean {
    const target = this.targets.get(id);
    if (!target) return false;
    
    if (!target.tags.includes(tag)) {
      target.tags.push(tag);
    }
    
    return true;
  }
}