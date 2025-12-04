export interface Vector3 {
    x: number;
    y: number;
    z: number;
}

export interface JointState {
    [jointName: string]: number;
}

export interface SimulationState {
    joints: JointState;
    position: Vector3;
    orientation: Vector3;
    velocity: Vector3;
    timestamp: number;
    metadata?: Record<string, any>;
}

export interface MotorCommand {
    jointId: string;
    value: number;
    duration?: number;
    powerLimit?: number;
}

export interface SensorReading {
    sensorId: string;
    value: number | boolean | Vector3;
    type: 'proximity' | 'force' | 'position' | 'orientation' | 'touch';
    timestamp: number;
}

export interface SimulationStep {
    stepId: string;
    motorCommands?: MotorCommand[];
    sensorReadings?: SensorReading[];
    duration: number;
    completionCondition?: CompletionCondition;
    metadata?: Record<string, any>;
}

export interface CompletionCondition {
    type: 'position_reached' | 'joint_state' | 'time_elapsed' | 'sensor_triggered';
    targetPosition?: Vector3;
    targetJoints?: JointState;
    timeout?: number;
    sensorId?: string;
    sensorValue?: any;
    tolerance?: number;
}

export interface TaskExecutionContext {
    currentStep: number;
    totalSteps: number;
    startState: SimulationState;
    currentState: SimulationState;
    targetState: SimulationState | null;
    constraints: RobotConstraints;
    capabilities: RobotCapability[];
}

export enum RobotCapability {
    ARM_MOVEMENT = 'arm_movement',
    LEGGED_LOCOMOTION = 'legged_locomotion',
    GRIPPER = 'gripper',
    FLIGHT = 'flight',
    SENSOR_FUSION = 'sensor_fusion',
    OBSTACLE_AVOIDANCE = 'obstacle_avoidance'
}

export interface RobotConstraints {
    maxVelocity: number;
    maxAcceleration: number;
    jointLimits: {
        [jointName: string]: {
            min: number;
            max: number;
        };
    };
    batteryCapacity?: number;
    payloadCapacity?: number;
    stepIntervalMs: number;
    stabilityThreshold?: number;
}

export interface SimulationProfile {
    name: string;
    type: 'arm' | 'humanoid' | 'drone' | 'wheeled';
    capabilities: RobotCapability[];
    initialState: SimulationState;
    constraints: RobotConstraints;
    computeNextState: (currentState: SimulationState, step: SimulationStep) => SimulationState;
}

export class ArmBotProfile implements SimulationProfile {
    name = 'ArmBot';
    type = 'arm' as const;
    
    capabilities = [
        RobotCapability.ARM_MOVEMENT,
        RobotCapability.GRIPPER,
        RobotCapability.SENSOR_FUSION
    ];

    initialState: SimulationState = {
        joints: {
            base: 0,
            shoulder: 0,
            elbow: 0,
            wrist: 0,
            wrist_rotate: 0,
            gripper: 0
        },
        position: { x: 0, y: 0, z: 0.5 },
        orientation: { x: 0, y: 0, z: 0 },
        velocity: { x: 0, y: 0, z: 0 },
        timestamp: Date.now(),
        metadata: { reach: 0.8, payload: 2.5 }
    };

    constraints: RobotConstraints = {
        maxVelocity: 1.5,
        maxAcceleration: 0.5,
        jointLimits: {
            base: { min: -180, max: 180 },
            shoulder: { min: -90, max: 90 },
            elbow: { min: 0, max: 135 },
            wrist: { min: -90, max: 90 },
            wrist_rotate: { min: -180, max: 180 },
            gripper: { min: 0, max: 1 }
        },
        batteryCapacity: 100,
        payloadCapacity: 2.5,
        stepIntervalMs: 100
    };

    computeNextState(currentState: SimulationState, step: SimulationStep): SimulationState {
        const nextState = { ...currentState };
        const deltaTime = step.duration / 1000;

        // Process motor commands
        if (step.motorCommands) {
            for (const command of step.motorCommands) {
                if (nextState.joints[command.jointId] !== undefined) {
                    const currentValue = nextState.joints[command.jointId];
                    const limit = this.constraints.jointLimits[command.jointId];
                    const maxChange = this.constraints.maxVelocity * deltaTime;
                    
                    let newValue = currentValue + (command.value * maxChange);
                    
                    // Apply joint limits
                    if (limit) {
                        newValue = Math.max(limit.min, Math.min(limit.max, newValue));
                    }
                    
                    // Apply power limit if specified
                    if (command.powerLimit && Math.abs(newValue - currentValue) > command.powerLimit * deltaTime) {
                        newValue = currentValue + (Math.sign(command.value) * command.powerLimit * deltaTime);
                    }
                    
                    nextState.joints[command.jointId] = newValue;
                }
            }
        }

        // Update position based on joint states 
        const shoulderAngle = nextState.joints.shoulder * (Math.PI / 180);
        const elbowAngle = nextState.joints.elbow * (Math.PI / 180);
        const armLength = 0.4;
        const forearmLength = 0.35;
        
        nextState.position.x = armLength * Math.cos(shoulderAngle) + forearmLength * Math.cos(shoulderAngle + elbowAngle);
        nextState.position.y = 0;
        nextState.position.z = 0.5 + armLength * Math.sin(shoulderAngle) + forearmLength * Math.sin(shoulderAngle + elbowAngle);

        // Update velocity 
        const timeDelta = (Date.now() - currentState.timestamp) / 1000;
        if (timeDelta > 0) {
            nextState.velocity.x = (nextState.position.x - currentState.position.x) / timeDelta;
            nextState.velocity.y = (nextState.position.y - currentState.position.y) / timeDelta;
            nextState.velocity.z = (nextState.position.z - currentState.position.z) / timeDelta;
        }

        nextState.timestamp = Date.now();
        return nextState;
    }
}

export class HumanoidProfile implements SimulationProfile {
    name = 'Humanoid';
    type = 'humanoid' as const;
    
    capabilities = [
        RobotCapability.LEGGED_LOCOMOTION,
        RobotCapability.ARM_MOVEMENT,
        RobotCapability.SENSOR_FUSION,
        RobotCapability.OBSTACLE_AVOIDANCE
    ];

    initialState: SimulationState = {
        joints: {
            head_yaw: 0,
            head_pitch: 0,
            left_shoulder: 0,
            left_elbow: 0,
            right_shoulder: 0,
            right_elbow: 0,
            left_hip_yaw: 0,
            left_hip_roll: 0,
            left_hip_pitch: 0,
            left_knee: 0,
            left_ankle: 0,
            right_hip_yaw: 0,
            right_hip_roll: 0,
            right_hip_pitch: 0,
            right_knee: 0,
            right_ankle: 0
        },
        position: { x: 0, y: 0, z: 0.9 },
        orientation: { x: 0, y: 0, z: 0 },
        velocity: { x: 0, y: 0, z: 0 },
        timestamp: Date.now(),
        metadata: { height: 1.6, weight: 50, stance: 'standing' }
    };

    constraints: RobotConstraints = {
        maxVelocity: 2.0,
        maxAcceleration: 0.3,
        jointLimits: {
            head_yaw: { min: -90, max: 90 },
            head_pitch: { min: -30, max: 30 },
            left_shoulder: { min: -90, max: 90 },
            left_elbow: { min: 0, max: 135 },
            right_shoulder: { min: -90, max: 90 },
            right_elbow: { min: 0, max: 135 },
            left_hip_yaw: { min: -45, max: 45 },
            left_hip_roll: { min: -30, max: 30 },
            left_hip_pitch: { min: -45, max: 45 },
            left_knee: { min: 0, max: 120 },
            left_ankle: { min: -30, max: 30 },
            right_hip_yaw: { min: -45, max: 45 },
            right_hip_roll: { min: -30, max: 30 },
            right_hip_pitch: { min: -45, max: 45 },
            right_knee: { min: 0, max: 120 },
            right_ankle: { min: -30, max: 30 }
        },
        batteryCapacity: 100,
        payloadCapacity: 10,
        stepIntervalMs: 50,
        stabilityThreshold: 0.8
    };

    computeNextState(currentState: SimulationState, step: SimulationStep): SimulationState {
        const nextState = { ...currentState };
        const deltaTime = step.duration / 1000;

        // Process locomotion commands
        if (step.motorCommands) {
            let forwardCommand = 0;
            let turnCommand = 0;

            for (const command of step.motorCommands) {
                if (command.jointId === 'locomotion_forward') {
                    forwardCommand = command.value;
                } else if (command.jointId === 'locomotion_turn') {
                    turnCommand = command.value;
                } else if (nextState.joints[command.jointId] !== undefined) {
                    const currentValue = nextState.joints[command.jointId];
                    const limit = this.constraints.jointLimits[command.jointId];
                    const maxChange = this.constraints.maxVelocity * deltaTime;
                    
                    let newValue = currentValue + (command.value * maxChange);
                    
                    if (limit) {
                        newValue = Math.max(limit.min, Math.min(limit.max, newValue));
                    }
                    
                    nextState.joints[command.jointId] = newValue;
                }
            }

            // Update position based on locomotion commands
            const speed = forwardCommand * this.constraints.maxVelocity * deltaTime;
            const turnRate = turnCommand * 45 * (Math.PI / 180) * deltaTime;
            
            const currentOrientation = nextState.orientation.z;
            nextState.orientation.z = (currentOrientation + turnRate) % (2 * Math.PI);
            
            nextState.position.x += speed * Math.cos(currentOrientation);
            nextState.position.y += speed * Math.sin(currentOrientation);
        }

        // Check stability 
        const leftFootPressure = Math.abs(nextState.joints.left_ankle) < 15 ? 1 : 0.5;
        const rightFootPressure = Math.abs(nextState.joints.right_ankle) < 15 ? 1 : 0.5;
        const stability = (leftFootPressure + rightFootPressure) / 2;

        if (stability < (this.constraints.stabilityThreshold || 0.7)) {
            nextState.metadata = { ...nextState.metadata, stability: 'low', warning: 'potential_fall' };
        } else {
            nextState.metadata = { ...nextState.metadata, stability: 'high' };
        }

        // Update velocity
        const timeDelta = (Date.now() - currentState.timestamp) / 1000;
        if (timeDelta > 0) {
            nextState.velocity.x = (nextState.position.x - currentState.position.x) / timeDelta;
            nextState.velocity.y = (nextState.position.y - currentState.position.y) / timeDelta;
            nextState.velocity.z = (nextState.position.z - currentState.position.z) / timeDelta;
        }

        nextState.timestamp = Date.now();
        return nextState;
    }
}

export class DroneLiteProfile implements SimulationProfile {
    name = 'Drone-lite';
    type = 'drone' as const;
    
    capabilities = [
        RobotCapability.FLIGHT,
        RobotCapability.SENSOR_FUSION,
        RobotCapability.OBSTACLE_AVOIDANCE
    ];

    initialState: SimulationState = {
        joints: {
            motor_front_left: 0,
            motor_front_right: 0,
            motor_back_left: 0,
            motor_back_right: 0,
            camera_tilt: 0
        },
        position: { x: 0, y: 0, z: 1 },
        orientation: { x: 0, y: 0, z: 0 },
        velocity: { x: 0, y: 0, z: 0 },
        timestamp: Date.now(),
        metadata: { altitude: 1, battery: 100, flight_mode: 'stable' }
    };

    constraints: RobotConstraints = {
        maxVelocity: 5.0,
        maxAcceleration: 2.0,
        jointLimits: {
            motor_front_left: { min: 0, max: 1 },
            motor_front_right: { min: 0, max: 1 },
            motor_back_left: { min: 0, max: 1 },
            motor_back_right: { min: 0, max: 1 },
            camera_tilt: { min: -90, max: 90 }
        },
        batteryCapacity: 100,
        payloadCapacity: 0.5,
        stepIntervalMs: 20
    };

    computeNextState(currentState: SimulationState, step: SimulationStep): SimulationState {
        const nextState = { ...currentState };
        const deltaTime = step.duration / 1000;

        // Process motor commands for drone control
        if (step.motorCommands) {
            let throttle = 0;
            let roll = 0;
            let pitch = 0;
            let yaw = 0;

            for (const command of step.motorCommands) {
                switch (command.jointId) {
                    case 'throttle':
                        throttle = command.value;
                        break;
                    case 'roll':
                        roll = command.value;
                        break;
                    case 'pitch':
                        pitch = command.value;
                        break;
                    case 'yaw':
                        yaw = command.value;
                        break;
                    default:
                        if (nextState.joints[command.jointId] !== undefined) {
                            nextState.joints[command.jointId] = command.value;
                        }
                }
            }

            // Simplified drone physics
            const maxThrust = 9.81 * 1.2;
            const thrust = (0.5 + throttle * 0.5) * maxThrust;
            const gravity = 9.81;
            
            // Calculate acceleration in body frame
            const accelerationX = pitch * this.constraints.maxAcceleration;
            const accelerationY = roll * this.constraints.maxAcceleration;
            const accelerationZ = thrust - gravity;

            // Transform to world frame 
            const cosYaw = Math.cos(nextState.orientation.z);
            const sinYaw = Math.sin(nextState.orientation.z);
            
            const worldAccelX = accelerationX * cosYaw - accelerationY * sinYaw;
            const worldAccelY = accelerationX * sinYaw + accelerationY * cosYaw;

            // Update velocity
            nextState.velocity.x += worldAccelX * deltaTime;
            nextState.velocity.y += worldAccelY * deltaTime;
            nextState.velocity.z += accelerationZ * deltaTime;

            // Clamp velocity
            const maxVel = this.constraints.maxVelocity;
            nextState.velocity.x = Math.max(-maxVel, Math.min(maxVel, nextState.velocity.x));
            nextState.velocity.y = Math.max(-maxVel, Math.min(maxVel, nextState.velocity.y));
            nextState.velocity.z = Math.max(-maxVel, Math.min(maxVel, nextState.velocity.z));

            // Update position
            nextState.position.x += nextState.velocity.x * deltaTime;
            nextState.position.y += nextState.velocity.y * deltaTime;
            nextState.position.z += nextState.velocity.z * deltaTime;

            // Update orientation
            nextState.orientation.z += yaw * 90 * (Math.PI / 180) * deltaTime;
            nextState.orientation.z = nextState.orientation.z % (2 * Math.PI);

            // Prevent going below ground
            if (nextState.position.z < 0.1) {
                nextState.position.z = 0.1;
                nextState.velocity.z = Math.max(0, nextState.velocity.z);
            }
        }

        // Update battery 
        if (nextState.metadata && nextState.metadata.battery) {
            const powerConsumption = 0.1 * deltaTime;
            nextState.metadata.battery = Math.max(0, nextState.metadata.battery - powerConsumption);
            
            if (nextState.metadata.battery < 20) {
                nextState.metadata.flight_mode = 'low_battery';
            }
        }

        nextState.timestamp = Date.now();
        return nextState;
    }
}

export function getProfile(profileName: string): SimulationProfile | null {
    switch (profileName) {
        case 'ArmBot':
            return new ArmBotProfile();
        case 'Humanoid':
            return new HumanoidProfile();
        case 'Drone-lite':
            return new DroneLiteProfile();
        default:
            return null;
    }
}

export function validateStepAgainstProfile(step: SimulationStep, profile: SimulationProfile): boolean {
    if (!step.motorCommands) return true;
    
    for (const command of step.motorCommands) {
        if (command.jointId && !profile.constraints.jointLimits[command.jointId]) {
            return false;
        }
        
        if (command.value && Math.abs(command.value) > 1) {
            return false;
        }
    }
    
    return true;
}