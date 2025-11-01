/**
 * Core constants for SynapseRobo SDK
 * Defines network endpoints, timeouts, and configuration defaults
 */

export const SOLANA_NETWORKS = {
  mainnet: 'https://api.mainnet-beta.solana.com',
  devnet: 'https://api.devnet.solana.com',
  testnet: 'https://api.testnet.solana.com',
  localnet: 'http://localhost:8899'
} as const;

export const DEFAULT_SOLANA_NETWORK = SOLANA_NETWORKS.devnet;

export const ROBOT_TYPES = {
  HUMANOID: 'humanoid',
  ARM_BOT: 'arm_bot',
  DRONE: 'drone',
  ROVER: 'rover',
  CUSTOM: 'custom'
} as const;

export const AI_MODELS = {
  GPT4_VISION: 'gpt-4-vision-preview',
  CLAUDE_3: 'claude-3-opus-20240229',
  LOCAL_LLM: 'local-llama',
  CUSTOM: 'custom'
} as const;

export const COMMAND_TYPES = {
  MOVEMENT: 'movement',
  MANIPULATION: 'manipulation',
  SENSOR_READ: 'sensor_read',
  AI_INFERENCE: 'ai_inference',
  SYSTEM: 'system',
  EMERGENCY: 'emergency'
} as const;

export const JOINT_NAMES = {
  SHOULDER_PITCH: 'shoulder_pitch',
  SHOULDER_ROLL: 'shoulder_roll',
  ELBOW: 'elbow',
  WRIST_YAW: 'wrist_yaw',
  WRIST_PITCH: 'wrist_pitch',
  WRIST_ROLL: 'wrist_roll',
  GRIPPER: 'gripper',
  WAIST: 'waist',
  HEAD_PAN: 'head_pan',
  HEAD_TILT: 'head_tilt'
} as const;

export const SENSOR_TYPES = {
  CAMERA: 'camera',
  DEPTH: 'depth',
  LIDAR: 'lidar',
  IMU: 'imu',
  FORCE_TORQUE: 'force_torque',
  TEMPERATURE: 'temperature',
  PROXIMITY: 'proximity'
} as const;

export const ERROR_CODES = {
  ROBOT_DISCONNECTED: 'ROBOT_DISCONNECTED',
  SOLANA_CONNECTION_FAILED: 'SOLANA_CONNECTION_FAILED',
  AI_INFERENCE_FAILED: 'AI_INFERENCE_FAILED',
  KINEMATICS_ERROR: 'KINEMATICS_ERROR',
  SAFETY_VIOLATION: 'SAFETY_VIOLATION',
  TIMEOUT: 'TIMEOUT',
  INSUFFICIENT_FUNDS: 'INSUFFICIENT_FUNDS',
  INVALID_COMMAND: 'INVALID_COMMAND'
} as const;

export const DEFAULT_TIMEOUTS = {
  ROBOT_COMMAND: 30000,
  AI_INFERENCE: 60000,
  SOLANA_TRANSACTION: 120000,
  SENSOR_READ: 5000,
  CONNECTION: 10000
} as const;

export const PRIORITY_LEVELS = {
  EMERGENCY: 100,
  HIGH: 75,
  NORMAL: 50,
  LOW: 25,
  BACKGROUND: 10
} as const;

export const SAFETY_LIMITS = {
  MAX_JOINT_VELOCITY: 2.0, // rad/s
  MAX_JOINT_ACCELERATION: 5.0, // rad/s²
  MAX_CARTESIAN_VELOCITY: 1.0, // m/s
  MAX_CARTESIAN_ACCELERATION: 2.0, // m/s²
  MAX_FORCE: 50.0, // N
  MAX_TORQUE: 10.0, // Nm
  WORKSPACE_RADIUS: 2.0, // meters
  MIN_TEMPERATURE: -10, // °C
  MAX_TEMPERATURE: 60 // °C
} as const;

export const EVENT_TYPES = {
  ROBOT_CONNECTED: 'robot_connected',
  ROBOT_DISCONNECTED: 'robot_disconnected',
  COMMAND_STARTED: 'command_started',
  COMMAND_COMPLETED: 'command_completed',
  COMMAND_FAILED: 'command_failed',
  AI_RESPONSE_RECEIVED: 'ai_response_received',
  SOLANA_TRANSACTION_CONFIRMED: 'solana_transaction_confirmed',
  SAFETY_VIOLATION: 'safety_violation',
  SENSOR_DATA: 'sensor_data',
  TASK_QUEUED: 'task_queued',
  TASK_COMPLETED: 'task_completed'
} as const;

export const LOG_LEVELS = {
  ERROR: 0,
  WARN: 1,
  INFO: 2,
  DEBUG: 3,
  TRACE: 4
} as const;

export const DEFAULT_CONFIG = {
  solana: {
    network: DEFAULT_SOLANA_NETWORK,
    commitment: 'confirmed',
    maxRetries: 3,
    retryDelay: 1000
  },
  robot: {
    simulation: true,
    timeout: DEFAULT_TIMEOUTS.ROBOT_COMMAND,
    maxVelocity: 0.5,
    safetyEnabled: true
  },
  ai: {
    model: AI_MODELS.LOCAL_LLM,
    temperature: 0.7,
    maxTokens: 1000,
    timeout: DEFAULT_TIMEOUTS.AI_INFERENCE
  },
  logging: {
    level: LOG_LEVELS.INFO,
    format: 'json',
    enableTelemetry: true
  }
} as const;

export const KINEMATICS_CONSTANTS = {
  GRAVITY: 9.81, // m/s²
  DEG_TO_RAD: Math.PI / 180,
  RAD_TO_DEG: 180 / Math.PI,
  MAX_ITERATIONS: 1000,
  CONVERGENCE_THRESHOLD: 0.001
} as const;

export const NETWORK_CONFIG = {
  MAX_RETRIES: 5,
  RETRY_DELAY: 1000,
  TIMEOUT: 30000,
  HEARTBEAT_INTERVAL: 5000,
  RECONNECT_ATTEMPTS: 10
} as const;

export const FILE_PATHS = {
  CONFIG: './config/synapse.json',
  LOGS: './logs/synapse.log',
  KEYPAIRS: './keys/',
  CACHE: './cache/',
  TEMP: './temp/'
} as const;

export const API_ENDPOINTS = {
  SYNAPSE_MAINNET: 'https://api.synapserobotics.xyz/v1',
  SYNAPSE_TESTNET: 'https://api-testnet.synapserobotics.xyz/v1',
  SYNAPSE_DEVNET: 'https://api-devnet.synapserobotics.xyz/v1',
  LOCAL_SIMULATOR: 'http://localhost:4000/api'
} as const;

export const MESSAGE_TYPES = {
  COMMAND: 'command',
  RESPONSE: 'response',
  ERROR: 'error',
  HEARTBEAT: 'heartbeat',
  TELEMETRY: 'telemetry',
  STATUS: 'status'
} as const;

export const TASK_STATES = {
  PENDING: 'pending',
  RUNNING: 'running',
  PAUSED: 'paused',
  COMPLETED: 'completed',
  FAILED: 'failed',
  CANCELLED: 'cancelled'
} as const;

export const VERIFICATION_LEVELS = {
  NONE: 'none',
  BASIC: 'basic',
  ADVANCED: 'advanced',
  CRITICAL: 'critical'
} as const;

export const COMPLIANCE_STANDARDS = {
  ISO_13482: 'iso_13482', // Robots and robotic devices
  ISO_10218: 'iso_10218', // Industrial robots
  IEC_61508: 'iec_61508', // Functional safety
  NIST_AI_RMF: 'nist_ai_rmf' // AI risk management
} as const;

export const CRYPTO_CONFIG = {
  HASH_ALGORITHM: 'sha256',
  SIGNATURE_ALGORITHM: 'ed25519',
  ENCRYPTION_ALGORITHM: 'aes-256-gcm',
  KEY_DERIVATION_ITERATIONS: 100000
} as const;

export const ROBOT_SPECIFICATIONS = {
  HUMANOID: {
    DEFAULT_HEIGHT: 1.75,
    DEFAULT_WEIGHT: 65.0,
    MAX_PAYLOAD: 5.0,
    OPERATING_RANGE: 10.0,
    BATTERY_CAPACITY: 2000
  },
  ARM_BOT: {
    DEFAULT_REACH: 1.2,
    DEFAULT_PAYLOAD: 3.0,
    PRECISION: 0.001,
    REPEATABILITY: 0.005
  },
  DRONE: {
    MAX_FLIGHT_TIME: 30,
    MAX_ALTITUDE: 120,
    MAX_SPEED: 15.0,
    OPERATING_RANGE: 2000
  }
} as const;

export const AI_CONTEXT_LIMITS = {
  MAX_CONTEXT_LENGTH: 8192,
  MAX_MEMORY_ITEMS: 100,
  MAX_HISTORY_LENGTH: 50,
  CONTEXT_TTL: 3600000 // 1 hour
} as const;

export const PERFORMANCE_METRICS = {
  LATENCY_THRESHOLD: 100, // ms
  THROUGHPUT_TARGET: 1000, // operations/second
  AVAILABILITY_TARGET: 0.999, // 99.9%
  RELIABILITY_TARGET: 0.99 // 99%
} as const;

export const SECURITY_PROTOCOLS = {
  MIN_PASSWORD_LENGTH: 12,
  SESSION_TIMEOUT: 3600000, // 1 hour
  MAX_LOGIN_ATTEMPTS: 5,
  TOKEN_EXPIRY: 86400000 // 24 hours
} as const;

export const DATA_FORMATS = {
  TIMESTAMP_FORMAT: 'YYYY-MM-DDTHH:mm:ss.SSSZ',
  DATE_FORMAT: 'YYYY-MM-DD',
  TIME_FORMAT: 'HH:mm:ss',
  JSON_INDENT: 2
} as const;

export const PHYSICAL_CONSTANTS = {
  SPEED_OF_LIGHT: 299792458, // m/s
  PLANCK_CONSTANT: 6.62607015e-34, // J·s
  BOLTZMANN_CONSTANT: 1.380649e-23, // J/K
  AVOGADRO_CONSTANT: 6.02214076e23 // mol⁻¹
} as const;

export const MATHEMATICAL_CONSTANTS = {
  PI: Math.PI,
  E: Math.E,
  GOLDEN_RATIO: 1.618033988749895,
  EULER_MASCHERONI: 0.5772156649015329
} as const;

export const ROBOTICS_STANDARDS = {
  ROS_VERSION: 'ROS2',
  URDF_VERSION: '1.0',
  SDF_VERSION: '1.7',
  OPENRAVE_VERSION: '0.9'
} as const;

export const BLOCKCHAIN_STANDARDS = {
  SOLANA_PROGRAM_VERSION: '0.3.0',
  SPL_TOKEN_VERSION: '0.3.0',
  ANCHOR_VERSION: '0.28.0',
  METAPLEX_VERSION: '2.0.0'
} as const;

export const DEPRECATION_WARNINGS = {
  LEGACY_API_ENDPOINTS: [
    '/v0/robots',
    '/v0/ai',
    '/v0/blockchain'
  ],
  SUPPORT_UNTIL: '2024-12-31'
} as const;

export const FEATURE_FLAGS = {
  ENABLE_AI_PIPELINE: true,
  ENABLE_BLOCKCHAIN_VERIFICATION: true,
  ENABLE_ADVANCED_LOGGING: true,
  ENABLE_TELEMETRY: true,
  ENABLE_SIMULATION_MODE: true,
  ENABLE_MULTI_ROBOT_COORDINATION: false, // Coming soon
  ENABLE_CLOUD_SYNC: false // Coming soon
} as const;