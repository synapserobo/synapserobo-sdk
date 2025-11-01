/**
 * Core TypeScript type definitions for SynapseRobo SDK
 * Comprehensive type safety for robotics, AI, and blockchain operations
 */

import { Connection, Keypair, Transaction, PublicKey, SendTransactionError } from '@solana/web3.js';
import { EventEmitter } from 'events';
import { 
  EVENT_TYPES, 
  ROBOT_TYPES, 
  COMMAND_TYPES, 
  PRIORITY_LEVELS, 
  ERROR_CODES, 
  SENSOR_TYPES, 
  AI_MODELS,
  SOLANA_NETWORKS,
  LOG_LEVELS,
  SAFETY_LIMITS,
  TASK_STATES,
  VERIFICATION_LEVELS,
  COMPLIANCE_STANDARDS
} from '../constants';

// Core SDK Types
export interface SDKConfig {
  solana: SolanaConfig;
  robot: RobotConfig;
  ai: AIConfig;
  logging: LoggingConfig;
}

export interface SolanaConfig {
  network: keyof typeof SOLANA_NETWORKS | string;
  commitment: 'processed' | 'confirmed' | 'finalized';
  maxRetries: number;
  retryDelay: number;
  wallet?: Keypair;
  timeout?: number;
  preflightCommitment?: 'processed' | 'confirmed' | 'finalized';
}

export interface RobotConfig {
  simulation: boolean;
  timeout: number;
  maxVelocity: number;
  safetyEnabled: boolean;
  type?: keyof typeof ROBOT_TYPES;
  hardwareInterface?: string;
  calibrationData?: CalibrationData;
}

export interface AIConfig {
  model: keyof typeof AI_MODELS | string;
  temperature: number;
  maxTokens: number;
  timeout: number;
  apiKey?: string;
  endpoint?: string;
  provider?: 'openai' | 'anthropic' | 'local' | 'custom';
  contextWindow?: number;
  maxRetries?: number;
}

export interface LoggingConfig {
  level: number;
  format: 'json' | 'text';
  enableTelemetry: boolean;
  enableAudit?: boolean;
  maxBufferSize?: number;
}

export interface CalibrationData {
  jointOffsets: Record<string, number>;
  sensorCalibration: Record<string, any>;
  kinematicParameters: KinematicParameters;
  lastCalibrated: Date;
}

export interface KinematicParameters {
  dhParameters?: DHParameter[];
  urdfParameters?: URDFParameter[];
  workspaceLimits: WorkspaceLimits;
}

export interface DHParameter {
  theta: number;
  d: number;
  a: number;
  alpha: number;
}

export interface URDFParameter {
  joint: string;
  origin: { x: number; y: number; z: number };
  axis: { x: number; y: number; z: number };
  limits: { lower: number; upper: number; effort: number; velocity: number };
}

export interface WorkspaceLimits {
  x: { min: number; max: number };
  y: { min: number; max: number };
  z: { min: number; max: number };
  roll: { min: number; max: number };
  pitch: { min: number; max: number };
  yaw: { min: number; max: number };
}

// Robotics Types
export interface JointState {
  name: string;
  position: number; // radians
  velocity: number; // rad/s
  effort: number; // Nm
  temperature: number; // °C
  limits: JointLimits;
  calibrated: boolean;
}

export interface JointLimits {
  min: number;
  max: number;
  maxVelocity: number;
  maxAcceleration: number;
  maxEffort: number;
}

export interface Pose {
  x: number; // meters
  y: number;
  z: number;
  roll: number; // radians
  pitch: number;
  yaw: number;
  frameId: string;
  timestamp: number;
}

export interface Twist {
  linear: { x: number; y: number; z: number }; // m/s
  angular: { x: number; y: number; z: number }; // rad/s
  frameId: string;
  timestamp: number;
}

export interface RobotState {
  connected: boolean;
  joints: Map<string, JointState>;
  pose: Pose;
  twist: Twist;
  battery: number; // percentage
  temperature: number; // °C
  errors: string[];
  warnings: string[];
  mode: 'manual' | 'autonomous' | 'emergency';
  safetyStatus: SafetyStatus;
  timestamp: number;
}

export interface SafetyStatus {
  emergencyStop: boolean;
  protectiveStop: boolean;
  reducedMode: boolean;
  violations: SafetyViolation[];
  lastViolation?: Date;
}

export interface SafetyViolation {
  type: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  joint?: string;
  value: number;
  limit: number;
  timestamp: Date;
}

export interface Command {
  id: string;
  type: keyof typeof COMMAND_TYPES;
  priority: number;
  params: Record<string, any>;
  timeout: number;
  createdAt: Date;
  metadata?: CommandMetadata;
}

export interface CommandMetadata {
  source: 'user' | 'ai' | 'system' | 'script';
  userId?: string;
  sessionId?: string;
  correlationId?: string;
  verificationLevel: keyof typeof VERIFICATION_LEVELS;
}

export interface CommandResult {
  commandId: string;
  success: boolean;
  result?: any;
  error?: string;
  errorCode?: keyof typeof ERROR_CODES;
  duration: number;
  timestamp: Date;
  metrics?: CommandMetrics;
}

export interface CommandMetrics {
  executionTime: number;
  processingTime: number;
  queueTime: number;
  retryCount: number;
  resourceUsage: ResourceUsage;
}

export interface ResourceUsage {
  cpu: number;
  memory: number;
  network: number;
  battery: number;
}

export interface MovementCommand extends Command {
  type: 'MOVEMENT';
  params: {
    targetPose?: Pose;
    targetJointPositions?: Record<string, number>;
    trajectory?: Pose[];
    velocity?: number;
    acceleration?: number;
    relative?: boolean;
    frameId?: string;
    obstacleAvoidance?: boolean;
  };
}

export interface ManipulationCommand extends Command {
  type: 'MANIPULATION';
  params: {
    action: 'grip' | 'release' | 'rotate' | 'lift' | 'place';
    target: string;
    force?: number;
    speed?: number;
    position?: Pose;
    objectId?: string;
    tool?: string;
  };
}

export interface SensorReadCommand extends Command {
  type: 'SENSOR_READ';
  params: {
    sensor: keyof typeof SENSOR_TYPES | string;
    duration?: number;
    frequency?: number;
    resolution?: any;
    format?: string;
  };
}

export interface AIInferenceCommand extends Command {
  type: 'AI_INFERENCE';
  params: {
    prompt: string;
    context?: RobotContext;
    model?: string;
    temperature?: number;
    maxTokens?: number;
    stream?: boolean;
  };
}

export interface SystemCommand extends Command {
  type: 'SYSTEM';
  params: {
    action: 'calibrate' | 'reset' | 'shutdown' | 'restart' | 'update';
    target?: string;
    parameters?: any;
  };
}

export interface EmergencyCommand extends Command {
  type: 'EMERGENCY';
  params: {
    action: 'stop' | 'pause' | 'resume' | 'safe';
    level: 'protective' | 'emergency' | 'warning';
  };
}

// AI Types
export interface AIRequest {
  prompt: string;
  context?: RobotContext;
  constraints?: string[];
  maxTokens?: number;
  temperature?: number;
  model?: string;
  stream?: boolean;
  tools?: AITool[];
  systemMessage?: string;
}

export interface AIResponse {
  id: string;
  action: AIAction;
  confidence: number;
  reasoning: string;
  alternatives: AIAction[];
  processingTime: number;
  model: string;
  usage: AIUsage;
  safetyScore: number;
}

export interface AIUsage {
  promptTokens: number;
  completionTokens: number;
  totalTokens: number;
  cost?: number;
}

export interface AIAction {
  type: string;
  parameters: Record<string, any>;
  preconditions: string[];
  postconditions: string[];
  safetyChecks: string[];
  confidence: number;
  estimatedDuration: number;
  requiredResources: string[];
}

export interface AITool {
  name: string;
  description: string;
  parameters: any; // JSON Schema
  required?: string[];
}

export interface RobotContext {
  state: RobotState;
  environment: EnvironmentState;
  task: TaskState;
  history: Command[];
  capabilities: string[];
  constraints: Constraint[];
  goals: Goal[];
}

export interface EnvironmentState {
  objects: EnvironmentObject[];
  hazards: Hazard[];
  constraints: Constraint[];
  map?: OccupancyGrid;
  timestamp: number;
}

export interface EnvironmentObject {
  id: string;
  type: string;
  pose: Pose;
  dimensions: { width: number; height: number; depth: number };
  properties: Record<string, any>;
  confidence: number;
  timestamp: number;
}

export interface Hazard {
  type: string;
  location: Pose;
  radius: number;
  severity: 'low' | 'medium' | 'high' | 'critical';
  description: string;
  timestamp: number;
}

export interface Constraint {
  type: string;
  condition: string;
  priority: number;
  parameters: any;
}

export interface Goal {
  id: string;
  description: string;
  priority: number;
  deadline?: Date;
  conditions: string[];
  rewards?: Reward[];
}

export interface Reward {
  type: string;
  value: number;
  conditions: string[];
}

export interface OccupancyGrid {
  resolution: number;
  width: number;
  height: number;
  origin: Pose;
  data: number[];
}

// Solana Types
export interface SolanaTransaction {
  transaction: Transaction;
  signers: Keypair[];
  description: string;
  metadata: Record<string, any>;
  feePayer?: PublicKey;
  computeUnits?: number;
  priorityFee?: number;
}

export interface TransactionResult {
  signature: string;
  slot: number;
  confirmationStatus: 'processed' | 'confirmed' | 'finalized';
  error?: string;
  metadata?: Record<string, any>;
  logs?: string[];
  fee?: number;
  timestamp: Date;
}

export interface WalletState {
  connected: boolean;
  publicKey: PublicKey | null;
  balance: number;
  network: string;
  tokens: TokenBalance[];
  lastUpdate: Date;
}

export interface TokenBalance {
  mint: PublicKey;
  amount: number;
  decimals: number;
  symbol?: string;
  name?: string;
}

export interface SolanaProgram {
  programId: PublicKey;
  name: string;
  version: string;
  methods: ProgramMethod[];
}

export interface ProgramMethod {
  name: string;
  accounts: AccountMeta[];
  args: any[];
  returns?: any;
}

export interface AccountMeta {
  pubkey: PublicKey;
  isSigner: boolean;
  isWritable: boolean;
}

// Task Management Types
export interface Task {
  id: string;
  name: string;
  description: string;
  priority: number;
  commands: Command[];
  dependencies: string[];
  timeout: number;
  retryPolicy: RetryPolicy;
  createdAt: Date;
  updatedAt: Date;
  metadata?: TaskMetadata;
  constraints: TaskConstraint[];
}

export interface TaskMetadata {
  createdBy: string;
  projectId?: string;
  tags: string[];
  estimatedDuration: number;
  estimatedCost?: number;
  compliance: ComplianceInfo;
}

export interface ComplianceInfo {
  standards: (keyof typeof COMPLIANCE_STANDARDS)[];
  certifications: string[];
  auditTrail: AuditEntry[];
}

export interface AuditEntry {
  action: string;
  user: string;
  timestamp: Date;
  details: any;
  signature?: string;
}

export interface TaskConstraint {
  type: 'resource' | 'temporal' | 'safety' | 'regulatory';
  condition: string;
  parameters: any;
}

export interface RetryPolicy {
  maxAttempts: number;
  backoffMultiplier: number;
  maxDelay: number;
  retryableErrors: string[];
  jitter: boolean;
}

export interface TaskState {
  status: keyof typeof TASK_STATES;
  currentCommandIndex: number;
  attempts: number;
  startTime?: Date;
  endTime?: Date;
  error?: string;
  results: any[];
  progress: number;
  metrics: TaskMetrics;
}

export interface TaskMetrics {
  totalCommands: number;
  completedCommands: number;
  failedCommands: number;
  totalDuration: number;
  averageCommandTime: number;
  resourceUsage: ResourceUsage;
  successRate: number;
}

// Pipeline Types
export interface PipelineStage {
  name: string;
  type: 'input' | 'processing' | 'output' | 'verification';
  handler: (input: any) => Promise<any>;
  timeout: number;
  retryable: boolean;
  dependencies: string[];
}

export interface PipelineConfig {
  stages: PipelineStage[];
  maxConcurrency: number;
  timeout: number;
  retryPolicy: RetryPolicy;
  monitoring: PipelineMonitoring;
}

export interface PipelineMonitoring {
  enabled: boolean;
  metrics: string[];
  alerts: PipelineAlert[];
  dashboard?: string;
}

export interface PipelineAlert {
  condition: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  action: 'log' | 'notify' | 'stop' | 'escalate';
  recipients: string[];
}

// Event System Types
export interface SDKEvent {
  type: keyof typeof EVENT_TYPES;
  timestamp: Date;
  data: any;
  source: string;
  correlationId?: string;
  severity: 'info' | 'warning' | 'error' | 'critical';
  metadata?: EventMetadata;
}

export interface EventMetadata {
  component: string;
  version: string;
  environment: string;
  user?: string;
  session?: string;
}

export interface EventListener {
  (event: SDKEvent): void;
}

export interface EventSubscription {
  id: string;
  eventType: string;
  listener: EventListener;
  options?: SubscriptionOptions;
}

export interface SubscriptionOptions {
  once?: boolean;
  filter?: (event: SDKEvent) => boolean;
  timeout?: number;
}

// Error Types
export interface SDKError extends Error {
  code: keyof typeof ERROR_CODES;
  details?: any;
  retryable: boolean;
  context: Record<string, any>;
  timestamp: Date;
  stack?: string;
}

export interface ValidationError {
  field: string;
  message: string;
  value: any;
  constraint: string;
}

export interface BusinessError extends SDKError {
  category: 'validation' | 'business' | 'technical' | 'security';
  userMessage?: string;
  recoverySteps?: string[];
}

// Hardware Types
export interface HardwareInterface {
  type: string;
  version: string;
  capabilities: string[];
  connected: boolean;
  status: HardwareStatus;
}

export interface HardwareStatus {
  online: boolean;
  lastSeen: Date;
  metrics: HardwareMetrics;
  errors: HardwareError[];
}

export interface HardwareMetrics {
  uptime: number;
  temperature: number;
  memoryUsage: number;
  cpuUsage: number;
  networkLatency: number;
}

export interface HardwareError {
  code: string;
  message: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  timestamp: Date;
  recoverable: boolean;
}

// Simulation Types
export interface SimulationConfig {
  physicsEngine: 'bullet' | 'ode' | 'dart';
  realtimeFactor: number;
  headless: boolean;
  environment: SimulationEnvironment;
  sensors: SimulationSensor[];
}

export interface SimulationEnvironment {
  name: string;
  gravity: number;
  lighting: LightingConfig;
  objects: SimulationObject[];
}

export interface LightingConfig {
  ambient: number;
  diffuse: number;
  specular: number;
  directional: boolean;
}

export interface SimulationObject {
  id: string;
  type: 'static' | 'dynamic' | 'kinematic';
  geometry: Geometry;
  material: Material;
  pose: Pose;
}

export interface Geometry {
  type: 'box' | 'sphere' | 'cylinder' | 'mesh';
  dimensions: any;
  meshUrl?: string;
}

export interface Material {
  color: { r: number; g: number; b: number; a: number };
  texture?: string;
  friction: number;
  restitution: number;
}

export interface SimulationSensor {
  type: keyof typeof SENSOR_TYPES;
  pose: Pose;
  parameters: any;
  noise: SensorNoise;
}

export interface SensorNoise {
  type: 'gaussian' | 'uniform' | 'none';
  mean: number;
  stddev: number;
}

// Security Types
export interface SecurityContext {
  user: User;
  permissions: Permission[];
  session: Session;
  environment: SecurityEnvironment;
}

export interface User {
  id: string;
  username: string;
  roles: string[];
  permissions: string[];
  attributes: Record<string, any>;
}

export interface Permission {
  resource: string;
  actions: string[];
  conditions?: any;
}

export interface Session {
  id: string;
  createdAt: Date;
  expiresAt: Date;
  lastActivity: Date;
  ip: string;
  userAgent: string;
}

export interface SecurityEnvironment {
  trusted: boolean;
  compliance: ComplianceInfo;
  threats: Threat[];
  controls: SecurityControl[];
}

export interface Threat {
  type: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  description: string;
  mitigation: string;
}

export interface SecurityControl {
  type: 'preventive' | 'detective' | 'corrective';
  name: string;
  description: string;
  status: 'enabled' | 'disabled' | 'testing';
}

// API Response Types
export interface APIResponse<T = any> {
  success: boolean;
  data?: T;
  error?: string;
  metadata?: {
    timestamp: Date;
    requestId: string;
    processingTime: number;
    pagination?: PaginationInfo;
  };
}

export interface PaginationInfo {
  page: number;
  pageSize: number;
  total: number;
  totalPages: number;
}

// Stream Types
export interface CommandStream {
  write(command: Command): boolean;
  end(): void;
  on(event: 'data', listener: (command: Command) => void): void;
  on(event: 'end', listener: () => void): void;
  on(event: 'error', listener: (error: Error) => void): void;
}

export interface AIStream {
  write(request: AIRequest): boolean;
  end(): void;
  on(event: 'data', listener: (response: AIResponse) => void): void;
  on(event: 'end', listener: () => void): void;
  on(event: 'error', listener: (error: Error) => void): void;
}

export interface DataStream {
  write(data: any): boolean;
  end(): void;
  on(event: 'data', listener: (data: any) => void): void;
  on(event: 'end', listener: () => void): void;
  on(event: 'error', listener: (error: Error) => void): void;
}

// Configuration Types
export interface HumanoidConfig {
  height: number;
  armLength: number;
  legLength: number;
  maxStepHeight: number;
  balanceEnabled: boolean;
  gaitParameters: GaitParameters;
  manipulationCapabilities: ManipulationCapability[];
}

export interface GaitParameters {
  stepLength: number;
  stepHeight: number;
  stepDuration: number;
  doubleSupportRatio: number;
  stabilityMargin: number;
}

export interface ManipulationCapability {
  arm: 'left' | 'right' | 'both';
  reach: number;
  payload: number;
  precision: number;
  tools: string[];
}

export interface ArmBotConfig {
  dof: number;
  reach: number;
  payload: number;
  precision: number;
  workspace: WorkspaceLimits;
  toolChanger: boolean;
  forceControl: boolean;
}

// Sensor Data Types
export interface SensorData {
  type: keyof typeof SENSOR_TYPES;
  timestamp: Date;
  values: number[] | number[][];
  units: string;
  quality: number;
  calibration: CalibrationInfo;
}

export interface CalibrationInfo {
  calibrated: boolean;
  calibrationDate?: Date;
  parameters: any;
  accuracy: number;
}

export interface CameraData extends SensorData {
  type: 'CAMERA';
  resolution: { width: number; height: number };
  format: string;
  data: Buffer;
  intrinsics: CameraIntrinsics;
  extrinsics: CameraExtrinsics;
}

export interface CameraIntrinsics {
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  distortion: number[];
}

export interface CameraExtrinsics {
  pose: Pose;
  frameId: string;
}

export interface IMUData extends SensorData {
  type: 'IMU';
  acceleration: { x: number; y: number; z: number };
  gyroscope: { x: number; y: number; z: number };
  magnetometer: { x: number; y: number; z: number };
  orientation: { w: number; x: number; y: number; z: number };
  temperature: number;
}

export interface LidarData extends SensorData {
  type: 'LIDAR';
  points: number[][];
  intensities: number[];
  ranges: number[];
  angles: number[];
  maxRange: number;
  minRange: number;
}

// Performance Monitoring Types
export interface PerformanceMetrics {
  latency: LatencyMetrics;
  throughput: ThroughputMetrics;
  resource: ResourceMetrics;
  quality: QualityMetrics;
}

export interface LatencyMetrics {
  average: number;
  p50: number;
  p95: number;
  p99: number;
  max: number;
}

export interface ThroughputMetrics {
  requestsPerSecond: number;
  bytesPerSecond: number;
  successRate: number;
  errorRate: number;
}

export interface ResourceMetrics {
  cpu: UsageMetrics;
  memory: UsageMetrics;
  disk: UsageMetrics;
  network: UsageMetrics;
}

export interface UsageMetrics {
  current: number;
  average: number;
  max: number;
  threshold: number;
}

export interface QualityMetrics {
  accuracy: number;
  precision: number;
  recall: number;
  f1Score: number;
  confidence: number;
}

// Export commonly used types for convenience
export type { 
  Connection, 
  Keypair, 
  Transaction, 
  PublicKey,
  EventEmitter 
} from respective modules;

// Re-export constants for type safety
export {
  EVENT_TYPES,
  ROBOT_TYPES, 
  COMMAND_TYPES, 
  PRIORITY_LEVELS, 
  ERROR_CODES, 
  SENSOR_TYPES, 
  AI_MODELS,
  SOLANA_NETWORKS,
  LOG_LEVELS,
  SAFETY_LIMITS,
  TASK_STATES,
  VERIFICATION_LEVELS,
  COMPLIANCE_STANDARDS
};