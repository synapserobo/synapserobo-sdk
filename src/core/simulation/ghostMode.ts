import { SimulationProfile, SimulationStep, SimulationState, TaskExecutionContext } from './profiles';
import { ExecutionTrace, TraceEntry, StepType, SimulationStatus } from './execructionTrace';
import { getProfile } from './profiles';

export interface GhostModeResult {
    success: boolean;
    timeline: GhostTimelineEntry[];
    finalState: SimulationState | null;
    trace: ExecutionTrace | null;
    errors: GhostModeError[];
    executionTime: number;
    stepsProcessed: number;
}

export interface GhostTimelineEntry {
    timestamp: number;
    stepIndex: number;
    state: SimulationState;
    motorCommands: any[];
    sensorReadings: any[];
    conditionMet?: boolean;
}

export interface GhostModeError {
    stepIndex: number;
    message: string;
    type: 'constraint_violation' | 'invalid_command' | 'runtime_error';
    recoverable: boolean;
}

export interface GhostModeOptions {
    maxSteps: number;
    stepIntervalMs: number;
    enableTrace: boolean;
    stopOnError: boolean;
    validationStrictness: 'low' | 'medium' | 'high';
}

export class GhostModeExecutor {
    private profile: SimulationProfile;
    private currentState: SimulationState;
    private context: TaskExecutionContext;
    private trace: ExecutionTrace;
    private options: GhostModeOptions;
    private timeline: GhostTimelineEntry[];
    private errors: GhostModeError[];

    constructor(profileName: string, options?: Partial<GhostModeOptions>) {
        const profile = getProfile(profileName);
        if (!profile) {
            throw new Error(`Profile ${profileName} not found`);
        }

        this.profile = profile;
        this.currentState = { ...profile.initialState };
        this.options = {
            maxSteps: 1000,
            stepIntervalMs: 0,
            enableTrace: true,
            stopOnError: true,
            validationStrictness: 'medium',
            ...options
        };

        this.context = {
            currentStep: 0,
            totalSteps: 0,
            startState: { ...profile.initialState },
            currentState: { ...profile.initialState },
            targetState: null,
            constraints: profile.constraints,
            capabilities: profile.capabilities
        };

        this.trace = new ExecutionTrace(`ghost_${Date.now()}`, this.options.enableTrace);
        this.timeline = [];
        this.errors = [];
    }

    public execute(taskSteps: SimulationStep[]): GhostModeResult {
        const startTime = Date.now();
        let stepsProcessed = 0;
        let shouldContinue = true;

        this.trace.initialize(this.profile.name);
        this.recordInitialState();

        for (let i = 0; i < taskSteps.length && shouldContinue; i++) {
            if (stepsProcessed >= this.options.maxSteps) {
                this.recordError(i, 'Maximum step count reached', 'runtime_error', false);
                shouldContinue = false;
                break;
            }

            const step = taskSteps[i];
            const stepResult = this.processStep(i, step);

            if (!stepResult.success && this.options.stopOnError) {
                shouldContinue = false;
            }

            if (stepResult.success) {
                stepsProcessed++;
            }

            // Apply step interval if specified
            if (this.options.stepIntervalMs > 0) {
                this.currentState.timestamp += this.options.stepIntervalMs;
            }
        }

        const endTime = Date.now();
        const success = this.errors.length === 0 || this.errors.every(e => e.recoverable);

        this.trace.finalize(
            success ? SimulationStatus.COMPLETED : SimulationStatus.ERROR,
            success ? 'ghost_execution_complete' : 'ghost_execution_failed'
        );

        return {
            success,
            timeline: this.timeline,
            finalState: { ...this.currentState },
            trace: this.options.enableTrace ? this.trace : null,
            errors: this.errors,
            executionTime: endTime - startTime,
            stepsProcessed
        };
    }

    private processStep(stepIndex: number, step: SimulationStep): { success: boolean; state?: SimulationState } {
        // Validate step against profile constraints
        if (!this.validateStep(stepIndex, step)) {
            return { success: false };
        }

        try {
            const previousState = { ...this.currentState };
            const nextState = this.profile.computeNextState(this.currentState, step);

            // Validate the resulting state
            if (!this.validateState(stepIndex, nextState)) {
                this.currentState = previousState;
                return { success: false };
            }

            this.currentState = nextState;
            this.context.currentStep = stepIndex + 1;
            this.context.currentState = { ...this.currentState };

            // Record timeline entry
            this.recordTimelineEntry(stepIndex, step, nextState);

            // Record trace
            if (this.options.enableTrace) {
                this.trace.appendStep({
                    type: StepType.STATE_UPDATE,
                    timestamp: Date.now(),
                    stepNumber: stepIndex,
                    data: {
                        motorCommands: step.motorCommands || [],
                        sensorReadings: step.sensorReadings || [],
                        newState: nextState,
                        taskStepIndex: stepIndex
                    }
                });

                // Check for completion conditions
                if (step.completionCondition) {
                    const conditionMet = this.evaluateCompletionCondition(nextState, step.completionCondition);
                    if (conditionMet) {
                        this.trace.appendStep({
                            type: StepType.CONDITION_MET,
                            timestamp: Date.now(),
                            stepNumber: stepIndex,
                            data: {
                                condition: step.completionCondition,
                                state: nextState
                            }
                        });
                    }
                }
            }

            return { success: true, state: nextState };
        } catch (error) {
            this.recordError(stepIndex, `Runtime error: ${error.message}`, 'runtime_error', false);
            return { success: false };
        }
    }

    private validateStep(stepIndex: number, step: SimulationStep): boolean {
        // Basic validation
        if (!step || typeof step !== 'object') {
            this.recordError(stepIndex, 'Invalid step object', 'invalid_command', false);
            return false;
        }

        if (step.duration <= 0) {
            this.recordError(stepIndex, 'Step duration must be positive', 'invalid_command', true);
            return false;
        }

        // Validate motor commands against profile
        if (step.motorCommands) {
            for (const command of step.motorCommands) {
                if (!command.jointId) {
                    this.recordError(stepIndex, 'Motor command missing jointId', 'invalid_command', true);
                    return false;
                }

                // Check if joint exists in profile
                if (!this.profile.constraints.jointLimits[command.jointId]) {
                    this.recordError(
                        stepIndex,
                        `Joint ${command.jointId} not found in profile`,
                        'invalid_command',
                        false
                    );
                    return false;
                }

                // Check value range
                if (Math.abs(command.value) > 1) {
                    this.recordError(
                        stepIndex,
                        `Motor command value ${command.value} out of range [-1, 1]`,
                        'invalid_command',
                        true
                    );
                    
                    if (this.options.validationStrictness === 'high') {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    private validateState(stepIndex: number, state: SimulationState): boolean {
        if (!state || typeof state !== 'object') {
            this.recordError(stepIndex, 'Invalid state object', 'constraint_violation', false);
            return false;
        }

        // Check required fields
        const requiredFields = ['joints', 'position', 'orientation', 'velocity', 'timestamp'];
        for (const field of requiredFields) {
            if (state[field] === undefined) {
                this.recordError(stepIndex, `State missing required field: ${field}`, 'constraint_violation', false);
                return false;
            }
        }

        // Validate joint limits
        for (const [jointName, value] of Object.entries(state.joints)) {
            const limit = this.profile.constraints.jointLimits[jointName];
            if (limit && (value < limit.min || value > limit.max)) {
                this.recordError(
                    stepIndex,
                    `Joint ${jointName} value ${value} outside limits [${limit.min}, ${limit.max}]`,
                    'constraint_violation',
                    false
                );
                return false;
            }
        }

        // Validate velocity limits
        const maxVel = this.profile.constraints.maxVelocity;
        const velMagnitude = Math.sqrt(
            state.velocity.x ** 2 +
            state.velocity.y ** 2 +
            state.velocity.z ** 2
        );

        if (velMagnitude > maxVel * 1.1) {
            this.recordError(
                stepIndex,
                `Velocity magnitude ${velMagnitude.toFixed(2)} exceeds maximum ${maxVel}`,
                'constraint_violation',
                true
            );
            
            if (this.options.validationStrictness !== 'low') {
                return false;
            }
        }

        return true;
    }

    private evaluateCompletionCondition(state: SimulationState, condition: any): boolean {
        if (!condition) return false;

        switch (condition.type) {
            case 'position_reached':
                if (!condition.targetPosition) return false;
                const dx = state.position.x - condition.targetPosition.x;
                const dy = state.position.y - condition.targetPosition.y;
                const dz = state.position.z - condition.targetPosition.z;
                const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
                return distance <= (condition.tolerance || 0.01);

            case 'joint_state':
                if (!condition.targetJoints) return false;
                for (const [joint, target] of Object.entries(condition.targetJoints)) {
                    if (Math.abs(state.joints[joint] - target) > (condition.tolerance || 0.01)) {
                        return false;
                    }
                }
                return true;

            case 'time_elapsed':
                return condition.timeout ? (Date.now() - state.timestamp) >= condition.timeout : false;

            case 'sensor_triggered':
                return condition.sensorValue !== undefined;

            default:
                return false;
        }
    }

    private recordTimelineEntry(stepIndex: number, step: SimulationStep, state: SimulationState): void {
        const entry: GhostTimelineEntry = {
            timestamp: Date.now(),
            stepIndex,
            state: { ...state },
            motorCommands: step.motorCommands ? [...step.motorCommands] : [],
            sensorReadings: step.sensorReadings ? [...step.sensorReadings] : []
        };

        if (step.completionCondition) {
            entry.conditionMet = this.evaluateCompletionCondition(state, step.completionCondition);
        }

        this.timeline.push(entry);
    }

    private recordError(stepIndex: number, message: string, type: GhostModeError['type'], recoverable: boolean): void {
        const error: GhostModeError = {
            stepIndex,
            message,
            type,
            recoverable
        };

        this.errors.push(error);

        if (this.options.enableTrace) {
            this.trace.appendStep({
                type: StepType.ERROR,
                timestamp: Date.now(),
                stepNumber: stepIndex,
                data: {
                    error: message,
                    errorType: type,
                    recoverable,
                    stepIndex
                }
            });
        }
    }

    private recordInitialState(): void {
        this.timeline.push({
            timestamp: Date.now(),
            stepIndex: -1,
            state: { ...this.currentState },
            motorCommands: [],
            sensorReadings: []
        });
    }

    public getCurrentState(): SimulationState {
        return { ...this.currentState };
    }

    public getTimeline(): GhostTimelineEntry[] {
        return [...this.timeline];
    }

    public getErrors(): GhostModeError[] {
        return [...this.errors];
    }

    public reset(): void {
        this.currentState = { ...this.profile.initialState };
        this.context.currentStep = 0;
        this.context.currentState = { ...this.profile.initialState };
        this.timeline = [];
        this.errors = [];
        
        if (this.options.enableTrace) {
            this.trace = new ExecutionTrace(`ghost_${Date.now()}`, true);
            this.trace.initialize(this.profile.name);
        }
    }
}

export function runInGhostMode(
    profileName: string,
    taskSteps: SimulationStep[],
    options?: Partial<GhostModeOptions>
): GhostModeResult {
    const executor = new GhostModeExecutor(profileName, options);
    return executor.execute(taskSteps);
}

export function analyzeGhostResult(result: GhostModeResult): AnalysisReport {
    const report: AnalysisReport = {
        success: result.success,
        totalSteps: result.timeline.length - 1,
        stepsWithErrors: result.errors.length,
        executionTimeMs: result.executionTime,
        finalPosition: result.finalState?.position || null,
        constraintViolations: result.errors.filter(e => e.type === 'constraint_violation').length,
        commandErrors: result.errors.filter(e => e.type === 'invalid_command').length,
        recommendations: []
    };

    // Generate recommendations
    if (result.errors.length > 0) {
        if (result.errors.some(e => e.type === 'constraint_violation' && !e.recoverable)) {
            report.recommendations.push('Review joint limits and velocity constraints');
        }
        
        if (result.errors.some(e => e.type === 'invalid_command')) {
            report.recommendations.push('Validate motor commands before execution');
        }
        
        if (result.executionTime > 10000) {
            report.recommendations.push('Consider optimizing task steps for faster execution');
        }
    }

    return report;
}

export interface AnalysisReport {
    success: boolean;
    totalSteps: number;
    stepsWithErrors: number;
    executionTimeMs: number;
    finalPosition: { x: number; y: number; z: number } | null;
    constraintViolations: number;
    commandErrors: number;
    recommendations: string[];
}

export function exportGhostTimeline(timeline: GhostTimelineEntry[], format: 'json' | 'csv'): string {
    if (format === 'json') {
        return JSON.stringify(timeline, null, 2);
    } else {
        //  CSV export
        const headers = ['timestamp', 'step', 'x', 'y', 'z', 'motor_commands', 'sensor_readings'];
        const rows = timeline.map(entry => [
            entry.timestamp,
            entry.stepIndex,
            entry.state.position.x.toFixed(3),
            entry.state.position.y.toFixed(3),
            entry.state.position.z.toFixed(3),
            entry.motorCommands.length,
            entry.sensorReadings.length
        ]);
        
        return [headers.join(','), ...rows.map(row => row.join(','))].join('\n');
    }
}