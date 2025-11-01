
# 📜 SynapseRobo SDK - API Reference

> **Technical API Documentation**

## 📚 Table of Contents

- [Core Classes](#core-classes)
- [Neural Engine API](#neural-engine-api)
- [Robot Controller API](#robot-controller-api)
- [Solana Trust Layer API](#solana-trust-layer-api)
- [Hardware Interfaces](#hardware-interfaces)
- [Data Types](#data-types)
- [Error Handling](#error-handling)
- [Configuration Reference](#configuration-reference)

## 🎯 Core Classes

### NeuralEngine

The central AI intelligence component responsible for real-time decision making, learning, and adaptation.

```typescript
class NeuralEngine {
  constructor(config: NeuralEngineConfig);
  
  // Core inference methods
  async infer(sensorData: MultiModalInput): Promise<ActionPlan>;
  async streamInference(sensorData: MultiModalInput): Observable<InferenceChunk>;
  
  // Learning and adaptation
  async trainLocal(experience: TrainingData): Promise<TrainingResult>;
  async updateModel(update: ModelUpdate): Promise<void>;
  async federatedSync(updates: FederatedUpdate[]): Promise<SyncResult>;
  
  // Model management
  getModelHash(): string;
  async loadModel(modelPath: string, options?: LoadOptions): Promise<void>;
  async exportModel(format: ModelFormat): Promise<ExportedModel>;
  
  // System management
  getStatus(): EngineStatus;
  async warmup(): Promise<void>;
  async shutdown(): Promise<void>;
}
```

### RobotController

Unified interface for robot hardware control, motion planning, and safety enforcement.

```typescript
class RobotController {
  constructor(robotType: RobotType, config: RobotConfig);
  
  // Robot management
  async connect(): Promise<ConnectionStatus>;
  async disconnect(): Promise<void>;
  async calibrate(options?: CalibrationOptions): Promise<CalibrationResult>;
  
  // Motion control
  async executeMotion(command: MotionCommand): Promise<ExecutionResult>;
  async planTrajectory(waypoints: Waypoint[]): Promise<TrajectoryPlan>;
  async followTrajectory(trajectory: TrajectoryPlan): Promise<FollowResult>;
  
  // State management
  getCurrentState(): RobotState;
  getJointStates(): JointState[];
  getBatteryStatus(): BatteryStatus;
  
  // Safety systems
  async emergencyStop(): Promise<void>;
  async setSafetyBounds(bounds: SafetyBounds): Promise<void>;
  async overrideSafety(override: SafetyOverride): Promise<void>;
}
```

### SolanaTrustLayer

Blockchain integration layer for cryptographic verification, action logging, and economic coordination.

```typescript
class SolanaTrustLayer {
  constructor(config: SolanaConfig);
  
  // Connection management
  async connect(wallet: Wallet): Promise<ConnectionResult>;
  async disconnect(): Promise<void>;
  getConnectionStatus(): ConnectionStatus;
  
  // Action verification
  async logAction(action: RobotAction): Promise<TransactionSignature>;
  async verifyAction(signature: TransactionSignature): Promise<VerificationResult>;
  async generateProof(action: RobotAction): Promise<CryptographicProof>;
  
  // Model synchronization
  async commitModelHash(modelHash: string, metrics: TrainingMetrics): Promise<CommitResult>;
  async getModelHistory(robotId: string): Promise<ModelHistory[]>;
  
  // Economic coordination
  async processPayment(payment: PaymentRequest): Promise<PaymentResult>;
  async getBalance(): Promise<BalanceInfo>;
  async createBounty(bounty: BountyCreation): Promise<BountyResult>;
}
```

## 🧠 Neural Engine API

### NeuralEngineConfig

```typescript
interface NeuralEngineConfig {
  // Model configuration
  modelType: 'reinforcement' | 'supervised' | 'hybrid';
  modelPath?: string;
  modelFormat: 'tensorflow' | 'onnx' | 'tensorrt';
  
  // Performance settings
  inferenceBatchSize: number;
  maxInferenceTime: number; // milliseconds
  quantization: 'int8' | 'fp16' | 'fp32';
  
  // Learning configuration
  localTraining: boolean;
  federatedLearning: boolean;
  learningRate: number;
  experienceBufferSize: number;
  
  // Safety constraints
  confidenceThreshold: number;
  safetyOverrides: boolean;
  maxDecisionComplexity: number;
}
```

### Core Methods

#### `infer(sensorData: MultiModalInput): Promise<ActionPlan>`

Performs real-time inference on multi-modal sensor data to generate action plans.

**Parameters:**
```typescript
interface MultiModalInput {
  vision?: ImageData | ImageData[];
  lidar?: PointCloud;
  imu?: IMUData;
  audio?: AudioBuffer;
  proprioception?: JointState[];
  timestamp: number;
}

interface ActionPlan {
  actions: RobotAction[];
  confidence: number;
  reasoning?: string;
  alternatives?: AlternativeAction[];
  constraints: ActionConstraints;
}
```

**Example:**
```typescript
const sensorData: MultiModalInput = {
  vision: cameraFrame,
  lidar: latestPointCloud,
  imu: currentOrientation,
  timestamp: Date.now()
};

const actionPlan = await neuralEngine.infer(sensorData);

if (actionPlan.confidence > 0.85) {
  await robotController.executeMotion(actionPlan.actions[0]);
}
```

#### `streamInference(sensorData: MultiModalInput): Observable<InferenceChunk>`

Streams inference results in real-time for low-latency applications.

**Example:**
```typescript
neuralEngine.streamInference(sensorData).subscribe({
  next: (chunk: InferenceChunk) => {
    // Handle partial inference results
    if (chunk.type === 'partial_plan') {
      robotController.previewMotion(chunk.action);
    }
  },
  complete: () => {
    console.log('Inference stream completed');
  }
});
```

#### `trainLocal(experience: TrainingData): Promise<TrainingResult>`

Performs on-device training using collected experience data.

**Parameters:**
```typescript
interface TrainingData {
  experiences: Experience[];
  labels?: TrainingLabel[];
  validationSplit: number;
  epochs: number;
}

interface TrainingResult {
  loss: number;
  accuracy: number;
  trainingTime: number;
  modelHash: string;
}
```

## 🤖 Robot Controller API

### RobotConfig

```typescript
interface RobotConfig {
  // Hardware configuration
  robotType: RobotType;
  controlFrequency: number; // Hz
  communicationProtocol: 'ros2' | 'custom' | 'ethercat';
  
  // Safety configuration
  safetyBounds: SafetyBounds;
  emergencyStopEnabled: boolean;
  collisionAvoidance: boolean;
  
  // Performance settings
  maxVelocity: number;
  maxAcceleration: number;
  controlMode: 'position' | 'velocity' | 'torque';
  
  // Calibration
  autoCalibration: boolean;
  calibrationRoutine: string;
}
```

### Core Methods

#### `executeMotion(command: MotionCommand): Promise<ExecutionResult>`

Executes a motion command with real-time safety monitoring.

**Parameters:**
```typescript
interface MotionCommand {
  type: 'joint_trajectory' | 'cartesian_move' | 'gripper_control';
  target: JointPosition | CartesianPose | GripperState;
  constraints?: MotionConstraints;
  duration?: number;
  blending?: 'none' | 'linear' | 'circular';
}

interface ExecutionResult {
  success: boolean;
  actualPath: TrajectoryPoint[];
  executionTime: number;
  energyUsed: number;
  safetyEvents: SafetyEvent[];
}
```

**Example:**
```typescript
const moveCommand: MotionCommand = {
  type: 'cartesian_move',
  target: {
    position: { x: 0.5, y: 0.2, z: 0.8 },
    orientation: { x: 0, y: 0, z: 0, w: 1 }
  },
  constraints: {
    maxVelocity: 0.5,
    maxAcceleration: 2.0,
    forceLimit: 20.0
  }
};

const result = await robotController.executeMotion(moveCommand);
if (!result.success) {
  console.warn('Motion execution failed:', result.safetyEvents);
}
```

#### `planTrajectory(waypoints: Waypoint[]): Promise<TrajectoryPlan>`

Generates an optimal trajectory through specified waypoints.

**Parameters:**
```typescript
interface Waypoint {
  position: Vector3;
  orientation?: Quaternion;
  velocity?: number;
  dwellTime?: number;
}

interface TrajectoryPlan {
  segments: TrajectorySegment[];
  totalTime: number;
  totalDistance: number;
  feasibility: 'feasible' | 'marginal' | 'infeasible';
  optimizationScore: number;
}
```

#### `followTrajectory(trajectory: TrajectoryPlan): Promise<FollowResult>`

Executes a pre-planned trajectory with real-time adjustments.

**Example:**
```typescript
const trajectory = await robotController.planTrajectory(waypoints);
if (trajectory.feasibility === 'feasible') {
  const followResult = await robotController.followTrajectory(trajectory);
  
  // Stream real-time progress
  followResult.progressStream.subscribe(progress => {
    console.log(`Progress: ${(progress * 100).toFixed(1)}%`);
  });
}
```

## ⛓️ Solana Trust Layer API

### SolanaConfig

```typescript
interface SolanaConfig {
  // Network configuration
  network: 'mainnet-beta' | 'devnet' | 'testnet' | 'local';
  rpcEndpoint: string;
  wsEndpoint: string;
  commitment: 'processed' | 'confirmed' | 'finalized';
  
  // Program configuration
  programId: string;
  actionRegistry: string;
  modelRegistry: string;
  
  // Transaction settings
  maxRetries: number;
  confirmationTimeout: number;
  priorityFee: number; // microLamports
}
```

### Core Methods

#### `logAction(action: RobotAction): Promise<TransactionSignature>`

Logs a robot action to the blockchain with cryptographic proof.

**Parameters:**
```typescript
interface RobotAction {
  type: string;
  parameters: any;
  robotId: string;
  timestamp: number;
  sensorContext?: SensorData;
  safetyChecks?: SafetyCheckResult[];
}

interface TransactionSignature {
  signature: string;
  slot: number;
  confirmationStatus: 'processed' | 'confirmed' | 'finalized';
}
```

**Example:**
```typescript
const action: RobotAction = {
  type: 'object_pickup',
  parameters: {
    objectId: 'item_789',
    position: { x: 0.5, y: 0.2, z: 0.1 },
    force: 15.0
  },
  robotId: 'synapse_bot_001',
  timestamp: Date.now(),
  safetyChecks: [{
    type: 'force_limit',
    passed: true,
    maxForce: 20.0
  }]
};

const signature = await trustLayer.logAction(action);
console.log(`Action logged: ${signature.signature}`);
```

#### `commitModelHash(modelHash: string, metrics: TrainingMetrics): Promise<CommitResult>`

Commits a model hash and training metrics to the blockchain.

**Parameters:**
```typescript
interface TrainingMetrics {
  accuracy: number;
  loss: number;
  trainingSamples: number;
  validationScore: number;
  trainingDuration: number;
}

interface CommitResult {
  signature: string;
  modelVersion: number;
  gasUsed: number;
}
```

#### `processPayment(payment: PaymentRequest): Promise<PaymentResult>`

Processes micro-payments for robotic services and tasks.

**Parameters:**
```typescript
interface PaymentRequest {
  recipient: string;
  amount: number; // SOL
  memo?: string;
  reference?: string;
  feePayer?: 'sender' | 'receiver';
}

interface PaymentResult {
  signature: string;
  finalAmount: number;
  fees: number;
  timestamp: number;
}
```

## 🔌 Hardware Interfaces

### MotorController

```typescript
class MotorController {
  constructor(config: MotorConfig);
  
  // Control methods
  async setPosition(position: number, options?: ControlOptions): Promise<void>;
  async setVelocity(velocity: number, options?: ControlOptions): Promise<void>;
  async setTorque(torque: number, options?: ControlOptions): Promise<void>;
  
  // State monitoring
  getPosition(): number;
  getVelocity(): number;
  getTorque(): number;
  getTemperature(): number;
  
  // Configuration
  async setPIDGains(gains: PIDGains): Promise<void>;
  async setLimits(limits: MotorLimits): Promise<void>;
}
```

### SensorInterface

```typescript
class SensorInterface {
  constructor(sensorType: SensorType, config: SensorConfig);
  
  // Data acquisition
  async read(): Promise<SensorData>;
  stream(sampleRate: number): Observable<SensorData>;
  
  // Calibration
  async calibrate(): Promise<CalibrationResult>;
  async setZero(): Promise<void>;
  
  // Configuration
  getConfig(): SensorConfig;
  async updateConfig(config: Partial<SensorConfig>): Promise<void>;
}
```

## 📊 Data Types

### Common Type Definitions

```typescript
// Vector and geometry types
interface Vector3 {
  x: number;
  y: number;
  z: number;
}

interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

interface Pose {
  position: Vector3;
  orientation: Quaternion;
}

// Robot state types
interface RobotState {
  pose: Pose;
  velocity: Vector3;
  acceleration: Vector3;
  jointStates: JointState[];
  batteryLevel: number;
  timestamp: number;
}

interface JointState {
  position: number;
  velocity: number;
  effort: number;
  temperature: number;
}

// Sensor data types
interface PointCloud {
  points: Vector3[];
  intensities?: number[];
  colors?: number[];
  timestamp: number;
}

interface IMUData {
  acceleration: Vector3;
  angularVelocity: Vector3;
  orientation: Quaternion;
  timestamp: number;
}
```

## 🚨 Error Handling

### Custom Error Types

```typescript
class RoboticsError extends Error {
  constructor(
    message: string,
    public code: ErrorCode,
    public context?: any
  ) {
    super(message);
    this.name = 'RoboticsError';
  }
}

enum ErrorCode {
  // Hardware errors
  HARDWARE_CONNECTION_FAILED = 'HARDWARE_001',
  SENSOR_TIMEOUT = 'HARDWARE_002',
  MOTOR_FAULT = 'HARDWARE_003',
  
  // Safety errors
  SAFETY_VIOLATION = 'SAFETY_001',
  EMERGENCY_STOP_ACTIVATED = 'SAFETY_002',
  COLLISION_DETECTED = 'SAFETY_003',
  
  // AI errors
  INFERENCE_FAILED = 'AI_001',
  MODEL_LOAD_ERROR = 'AI_002',
  CONFIDENCE_TOO_LOW = 'AI_003',
  
  // Blockchain errors
  TRANSACTION_FAILED = 'BLOCKCHAIN_001',
  INSUFFICIENT_FUNDS = 'BLOCKCHAIN_002',
  NETWORK_ERROR = 'BLOCKCHAIN_003'
}
```

### Error Handling Patterns

```typescript
// Example: Safe motion execution with error handling
async function safeMotionExecution(command: MotionCommand): Promise<ExecutionResult> {
  try {
    // Validate command
    const validation = await robotController.validateCommand(command);
    if (!validation.valid) {
      throw new RoboticsError(
        `Command validation failed: ${validation.reason}`,
        ErrorCode.SAFETY_VIOLATION,
        { command, validation }
      );
    }
    
    // Execute with timeout
    const result = await Promise.race([
      robotController.executeMotion(command),
      new Promise<never>((_, reject) => 
        setTimeout(() => reject(new RoboticsError(
          'Motion execution timeout',
          ErrorCode.HARDWARE_CONNECTION_FAILED
        )), 5000)
      )
    ]);
    
    return result;
    
  } catch (error) {
    if (error instanceof RoboticsError) {
      // Log to blockchain for audit
      await trustLayer.logError({
        error: error.code,
        message: error.message,
        context: error.context,
        timestamp: Date.now()
      });
    }
    
    // Trigger safety protocols
    await robotController.emergencyStop();
    throw error;
  }
}
```

## ⚙️ Configuration Reference

### Complete Configuration Schema

```typescript
interface SynapseRoboConfig {
  version: string;
  
  ai: {
    engine: NeuralEngineConfig;
    models: {
      navigation: ModelConfig;
      manipulation: ModelConfig;
      perception: ModelConfig;
    };
  };
  
  robot: {
    controller: RobotConfig;
    hardware: {
      motors: MotorConfig[];
      sensors: SensorConfig[];
      communication: CommConfig;
    };
  };
  
  blockchain: {
    solana: SolanaConfig;
    programs: {
      actionRegistry: ProgramConfig;
      modelRegistry: ProgramConfig;
      economic: ProgramConfig;
    };
  };
  
  telemetry: {
    enabled: boolean;
    sampleRate: number;
    compression: 'none' | 'gzip' | 'lz4';
    encryption: boolean;
  };
  
  safety: {
    emergencyStop: EmergencyConfig;
    bounds: SafetyBounds;
    monitoring: MonitoringConfig;
  };
}
```

### Environment Variables

```bash
# Required environment variables
SYNAPSE_ROBO_NETWORK=mainnet-beta
SYNAPSE_ROBO_RPC_ENDPOINT=https://api.mainnet-beta.solana.com
SYNAPSE_ROBO_WALLET_KEY=your_wallet_private_key

# Optional configuration
SYNAPSE_ROBO_LOG_LEVEL=info
SYNAPSE_ROBO_TELEMETRY_ENABLED=true
SYNAPSE_ROBO_SAFETY_MODE=strict
```

---

## 🔗 Quick Reference

### Common Method Patterns

```typescript
// Standard robot operation pattern
const sensorData = await sensorInterface.read();
const actionPlan = await neuralEngine.infer(sensorData);
const executionResult = await robotController.executeMotion(actionPlan.actions[0]);
const verification = await trustLayer.logAction({
  type: actionPlan.actions[0].type,
  parameters: actionPlan.actions[0].parameters,
  robotId: robotController.getId(),
  timestamp: Date.now()
});

// Learning and adaptation pattern
const trainingData = await experienceBuffer.sample(1000);
const trainingResult = await neuralEngine.trainLocal(trainingData);
await trustLayer.commitModelHash(
  neuralEngine.getModelHash(),
  trainingResult.metrics
);
```

### Performance Tips

- Use `streamInference()` for real-time applications requiring low latency
- Batch sensor readings when possible to reduce context switching
- Pre-warm neural models before critical operations
- Use transaction batching for high-frequency blockchain operations
- Implement circuit breakers for hardware communication

---

**Need Help?**
- Check [Examples](../examples/) for practical implementations

**Found a Bug?** 
- Please report issues at [GitHub Issues](https://github.com/synapserobo/synapserobo-sdk/issues)

---