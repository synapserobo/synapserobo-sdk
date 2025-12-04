import { SimulationProfile, SimulationState, RobotCapability, SimulationStep, TaskExecutionContext } from './profiles';
import { ExecutionTrace, TraceEntry, SimulationStatus, StepType } from './executionTrace';
import { runInGhostMode, GhostModeResult } from './ghostMode';

export enum SimulationSessionState {
    IDLE = 'idle',
    RUNNING = 'running',
    PAUSED = 'paused',
    COMPLETED = 'completed',
    ERROR = 'error'
}

export interface SimulationSession {
    id: string;
    state: SimulationSessionState;
    profile: SimulationProfile | null;
    trace: ExecutionTrace;
    startTime: number | null;
    endTime: number | null;
    currentStep: number;
    taskContext: TaskExecutionContext | null;
    loopInterval: NodeJS.Timeout | null;
}

export interface SimulationOptions {
    stepIntervalMs: number;
    maxSteps: number;
    enableTrace: boolean;
    autoStart: boolean;
}

export class SimulationManager {
    private activeSessions: Map<string, SimulationSession>;
    private defaultOptions: SimulationOptions;
    private sessionCounter: number;

    constructor(options?: Partial<SimulationOptions>) {
        this.activeSessions = new Map();
        this.sessionCounter = 0;
        this.defaultOptions = {
            stepIntervalMs: 100,
            maxSteps: 1000,
            enableTrace: true,
            autoStart: true,
            ...options
        };
    }

    public createSession(profileName: string, options?: Partial<SimulationOptions>): string {
        const sessionId = `sim_${Date.now()}_${++this.sessionCounter}`;
        const mergedOptions = { ...this.defaultOptions, ...options };
        
        const session: SimulationSession = {
            id: sessionId,
            state: SimulationSessionState.IDLE,
            profile: null,
            trace: new ExecutionTrace(sessionId, mergedOptions.enableTrace),
            startTime: null,
            endTime: null,
            currentStep: 0,
            taskContext: null,
            loopInterval: null
        };

        this.activeSessions.set(sessionId, session);
        
        // Load profile if provided
        if (profileName) {
            this.loadProfile(sessionId, profileName);
        }

        return sessionId;
    }

    public loadProfile(sessionId: string, profileName: string): boolean {
        const session = this.activeSessions.get(sessionId);
        if (!session) {
            console.error(`Session ${sessionId} not found`);
            return false;
        }

        const profile = this.getProfileByName(profileName);
        if (!profile) {
            session.state = SimulationSessionState.ERROR;
            console.error(`Profile ${profileName} not found`);
            return false;
        }

        session.profile = profile;
        session.taskContext = {
            currentStep: 0,
            totalSteps: 0,
            startState: profile.initialState,
            currentState: { ...profile.initialState },
            targetState: null,
            constraints: profile.constraints,
            capabilities: profile.capabilities
        };

        session.trace.initialize(profileName);
        return true;
    }

    public startTask(sessionId: string, taskSteps: SimulationStep[]): boolean {
        const session = this.activeSessions.get(sessionId);
        if (!session || !session.profile || !session.taskContext) {
            console.error(`Cannot start task: invalid session or missing profile`);
            return false;
        }

        if (session.state !== SimulationSessionState.IDLE && session.state !== SimulationSessionState.PAUSED) {
            console.error(`Cannot start task from state: ${session.state}`);
            return false;
        }

        session.taskContext.totalSteps = taskSteps.length;
        session.taskContext.targetState = null;
        session.startTime = Date.now();
        session.state = SimulationSessionState.RUNNING;

        // Start simulation loop
        this.startSimulationLoop(sessionId, taskSteps);
        return true;
    }

    private startSimulationLoop(sessionId: string, taskSteps: SimulationStep[]): void {
        const session = this.activeSessions.get(sessionId);
        if (!session) return;

        const stepInterval = session.profile?.constraints?.stepIntervalMs || this.defaultOptions.stepIntervalMs;
        
        session.loopInterval = setInterval(() => {
            this.step(sessionId, taskSteps);
        }, stepInterval);
    }

    public step(sessionId: string, taskSteps: SimulationStep[]): boolean {
        const session = this.activeSessions.get(sessionId);
        if (!session || !session.profile || !session.taskContext) {
            return false;
        }

        if (session.state !== SimulationSessionState.RUNNING) {
            return false;
        }

        // Check max steps
        if (session.currentStep >= this.defaultOptions.maxSteps) {
            this.completeSession(sessionId, 'max_steps_reached');
            return false;
        }

        // Check if all task steps are processed
        if (session.taskContext.currentStep >= taskSteps.length) {
            this.completeSession(sessionId, 'task_completed');
            return false;
        }

        const currentTaskStep = taskSteps[session.taskContext.currentStep];
        const nextState = session.profile.computeNextState(
            session.taskContext.currentState,
            currentTaskStep
        );

        // Validate state against constraints
        const isValid = this.validateState(nextState, session.profile.constraints);
        if (!isValid) {
            session.trace.appendStep({
                type: StepType.CONSTRAINT_VIOLATION,
                timestamp: Date.now(),
                stepNumber: session.currentStep,
                data: {
                    state: nextState,
                    constraint: session.profile.constraints,
                    message: 'State violates robot constraints'
                }
            });
            this.completeSession(sessionId, 'constraint_violation');
            return false;
        }

        // Update session state
        session.taskContext.currentState = nextState;
        session.taskContext.currentStep++;
        session.currentStep++;

        // Record trace
        session.trace.appendStep({
            type: StepType.STATE_UPDATE,
            timestamp: Date.now(),
            stepNumber: session.currentStep,
            data: {
                motorCommands: currentTaskStep.motorCommands || [],
                sensorReadings: currentTaskStep.sensorReadings || [],
                newState: nextState,
                taskStepIndex: session.taskContext.currentStep - 1
            }
        });

        // Check for completion conditions
        if (currentTaskStep.completionCondition) {
            const isComplete = this.evaluateCompletionCondition(
                nextState,
                currentTaskStep.completionCondition
            );
            if (isComplete) {
                session.trace.appendStep({
                    type: StepType.CONDITION_MET,
                    timestamp: Date.now(),
                    stepNumber: session.currentStep,
                    data: {
                        condition: currentTaskStep.completionCondition,
                        state: nextState
                    }
                });
            }
        }

        return true;
    }

    public pause(sessionId: string): boolean {
        const session = this.activeSessions.get(sessionId);
        if (!session || session.state !== SimulationSessionState.RUNNING) {
            return false;
        }

        if (session.loopInterval) {
            clearInterval(session.loopInterval);
            session.loopInterval = null;
        }

        session.state = SimulationSessionState.PAUSED;
        session.trace.appendStep({
            type: StepType.SESSION_PAUSED,
            timestamp: Date.now(),
            stepNumber: session.currentStep,
            data: { reason: 'user_request' }
        });

        return true;
    }

    public resume(sessionId: string): boolean {
        const session = this.activeSessions.get(sessionId);
        if (!session || session.state !== SimulationSessionState.PAUSED || !session.taskContext) {
            return false;
        }

        session.state = SimulationSessionState.RUNNING;
        
        session.trace.appendStep({
            type: StepType.SESSION_RESUMED,
            timestamp: Date.now(),
            stepNumber: session.currentStep,
            data: { reason: 'user_request' }
        });

        return true;
    }

    public stop(sessionId: string, reason?: string): boolean {
        const session = this.activeSessions.get(sessionId);
        if (!session) {
            return false;
        }

        if (session.loopInterval) {
            clearInterval(session.loopInterval);
            session.loopInterval = null;
        }

        session.state = SimulationSessionState.COMPLETED;
        session.endTime = Date.now();

        if (reason) {
            session.trace.appendStep({
                type: StepType.SESSION_STOPPED,
                timestamp: Date.now(),
                stepNumber: session.currentStep,
                data: { reason }
            });
        }

        session.trace.finalize(session.state, reason || 'manual_stop');
        return true;
    }

    public getSessionState(sessionId: string): SimulationSessionState | null {
        const session = this.activeSessions.get(sessionId);
        return session ? session.state : null;
    }

    public getTrace(sessionId: string): ExecutionTrace | null {
        const session = this.activeSessions.get(sessionId);
        return session ? session.trace : null;
    }

    public runGhostMode(profileName: string, taskSteps: SimulationStep[]): GhostModeResult {
        return runInGhostMode(profileName, taskSteps, this.defaultOptions);
    }

    private completeSession(sessionId: string, reason: string): void {
        const session = this.activeSessions.get(sessionId);
        if (!session) return;

        if (session.loopInterval) {
            clearInterval(session.loopInterval);
            session.loopInterval = null;
        }

        session.state = SimulationSessionState.COMPLETED;
        session.endTime = Date.now();
        session.trace.finalize(session.state, reason);
    }

    private validateState(state: SimulationState, constraints: any): boolean {
        // Basic validation
        if (!state || typeof state !== 'object') return false;
        
        // Check for required fields
        if (state.joints === undefined || state.position === undefined) {
            return false;
        }

        // Validate joint ranges if constraints exist
        if (constraints?.jointLimits && state.joints) {
            for (const [jointName, value] of Object.entries(state.joints)) {
                const limit = constraints.jointLimits[jointName];
                if (limit && (value < limit.min || value > limit.max)) {
                    return false;
                }
            }
        }

        return true;
    }

    private evaluateCompletionCondition(state: SimulationState, condition: any): boolean {    
        if (!condition || typeof condition !== 'object') return false;

        if (condition.type === 'position_reached' && condition.targetPosition) {
            const distance = Math.sqrt(
                Math.pow(state.position.x - condition.targetPosition.x, 2) +
                Math.pow(state.position.y - condition.targetPosition.y, 2) +
                Math.pow(state.position.z - condition.targetPosition.z, 2)
            );
            return distance <= (condition.tolerance || 0.01);
        }

        if (condition.type === 'joint_state' && condition.targetJoints) {
            for (const [joint, target] of Object.entries(condition.targetJoints)) {
                if (Math.abs(state.joints[joint] - target) > (condition.tolerance || 0.01)) {
                    return false;
                }
            }
            return true;
        }

        return false;
    }

    private getProfileByName(profileName: string): SimulationProfile | null {
        const profiles: Record<string, SimulationProfile> = {
            'ArmBot': {
                name: 'ArmBot',
                type: 'arm',
                capabilities: [RobotCapability.ARM_MOVEMENT, RobotCapability.GRIPPER],
                initialState: {
                    joints: { base: 0, shoulder: 0, elbow: 0, wrist: 0, gripper: 0 },
                    position: { x: 0, y: 0, z: 0 },
                    orientation: { x: 0, y: 0, z: 0 },
                    velocity: { x: 0, y: 0, z: 0 },
                    timestamp: Date.now()
                },
                constraints: {
                    maxVelocity: 1.0,
                    jointLimits: {
                        base: { min: -180, max: 180 },
                        shoulder: { min: -90, max: 90 },
                        elbow: { min: 0, max: 135 },
                        wrist: { min: -90, max: 90 },
                        gripper: { min: 0, max: 1 }
                    },
                    stepIntervalMs: 100
                },
                computeNextState: (state: SimulationState, step: SimulationStep) => {
                    const nextState = { ...state };
                    if (step.motorCommands) {
                        step.motorCommands.forEach(cmd => {
                            if (nextState.joints[cmd.jointId] !== undefined) {
                                nextState.joints[cmd.jointId] += cmd.value * 0.1;
                            }
                        });
                    }
                    nextState.timestamp = Date.now();
                    return nextState;
                }
            }
        };

        return profiles[profileName] || null;
    }

    public cleanupSession(sessionId: string): void {
        const session = this.activeSessions.get(sessionId);
        if (session && session.loopInterval) {
            clearInterval(session.loopInterval);
        }
        this.activeSessions.delete(sessionId);
    }

    public getActiveSessions(): string[] {
        return Array.from(this.activeSessions.keys());
    }
}