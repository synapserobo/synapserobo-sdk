export enum StepType {
    STATE_UPDATE = 'state_update',
    SENSOR_TRIGGER = 'sensor_trigger',
    MOTOR_COMMAND = 'motor_command',
    CONSTRAINT_VIOLATION = 'constraint_violation',
    CONDITION_MET = 'condition_met',
    ERROR = 'error',
    WARNING = 'warning',
    SESSION_STARTED = 'session_started',
    SESSION_PAUSED = 'session_paused',
    SESSION_RESUMED = 'session_resumed',
    SESSION_STOPPED = 'session_stopped'
}

export enum SimulationStatus {
    IDLE = 'idle',
    RUNNING = 'running',
    PAUSED = 'paused',
    COMPLETED = 'completed',
    ERROR = 'error',
    CANCELLED = 'cancelled'
}

export interface TraceEntry {
    type: StepType;
    timestamp: number;
    stepNumber: number;
    data: Record<string, any>;
    sessionId?: string;
    profile?: string;
}

export interface TraceSummary {
    totalSteps: number;
    duration: number;
    startTime: number;
    endTime: number;
    finalStatus: SimulationStatus;
    errors: number;
    warnings: number;
    constraintViolations: number;
    conditionsMet: number;
    motorCommands: number;
    sensorTriggers: number;
}

export interface TraceExportOptions {
    includeData: boolean;
    format: 'json' | 'minimal' | 'debug';
    filterTypes?: StepType[];
    startStep?: number;
    endStep?: number;
}

export class ExecutionTrace {
    private entries: TraceEntry[];
    private sessionId: string;
    private profileName: string | null;
    private startTime: number | null;
    private endTime: number | null;
    private status: SimulationStatus;
    private enableLogging: boolean;
    private summary: TraceSummary;

    constructor(sessionId: string, enableLogging: boolean = true) {
        this.entries = [];
        this.sessionId = sessionId;
        this.profileName = null;
        this.startTime = null;
        this.endTime = null;
        this.status = SimulationStatus.IDLE;
        this.enableLogging = enableLogging;
        
        this.summary = {
            totalSteps: 0,
            duration: 0,
            startTime: 0,
            endTime: 0,
            finalStatus: SimulationStatus.IDLE,
            errors: 0,
            warnings: 0,
            constraintViolations: 0,
            conditionsMet: 0,
            motorCommands: 0,
            sensorTriggers: 0
        };
    }

    public initialize(profileName: string): void {
        this.profileName = profileName;
        this.startTime = Date.now();
        this.status = SimulationStatus.RUNNING;
        
        this.summary.startTime = this.startTime;
        this.summary.finalStatus = this.status;

        this.append({
            type: StepType.SESSION_STARTED,
            timestamp: this.startTime,
            stepNumber: 0,
            data: {
                profile: profileName,
                sessionId: this.sessionId,
                timestamp: this.startTime
            }
        });
    }

    public appendStep(entry: Omit<TraceEntry, 'sessionId' | 'profile'>): void {
        const fullEntry: TraceEntry = {
            ...entry,
            sessionId: this.sessionId,
            profile: this.profileName || undefined
        };

        this.entries.push(fullEntry);
        this.updateSummary(fullEntry);

        if (this.enableLogging) {
            this.logEntry(fullEntry);
        }
    }

    public append(entry: TraceEntry): void {
        this.entries.push(entry);
        this.updateSummary(entry);

        if (this.enableLogging) {
            this.logEntry(entry);
        }
    }

    private updateSummary(entry: TraceEntry): void {
        this.summary.totalSteps++;

        switch (entry.type) {
            case StepType.ERROR:
                this.summary.errors++;
                break;
            case StepType.WARNING:
                this.summary.warnings++;
                break;
            case StepType.CONSTRAINT_VIOLATION:
                this.summary.constraintViolations++;
                break;
            case StepType.CONDITION_MET:
                this.summary.conditionsMet++;
                break;
            case StepType.MOTOR_COMMAND:
                this.summary.motorCommands++;
                break;
            case StepType.SENSOR_TRIGGER:
                this.summary.sensorTriggers++;
                break;
        }
    }

    private logEntry(entry: TraceEntry): void {
        const timestamp = new Date(entry.timestamp).toISOString().split('T')[1].slice(0, -1);
        const prefix = `[${timestamp}] [${entry.type}]`;
        
        switch (entry.type) {
            case StepType.STATE_UPDATE:
                const state = entry.data.newState;
                console.log(`${prefix} Step ${entry.stepNumber}: Position (${state.position.x.toFixed(2)}, ${state.position.y.toFixed(2)}, ${state.position.z.toFixed(2)})`);
                break;
                
            case StepType.ERROR:
                console.error(`${prefix} Step ${entry.stepNumber}: ${entry.data.error}`);
                break;
                
            case StepType.CONSTRAINT_VIOLATION:
                console.warn(`${prefix} Step ${entry.stepNumber}: Constraint violation - ${entry.data.message}`);
                break;
                
            case StepType.CONDITION_MET:
                console.log(`${prefix} Step ${entry.stepNumber}: Completion condition met`);
                break;
                
            default:
                console.log(`${prefix} Step ${entry.stepNumber}`);
        }
    }

    public finalize(finalStatus: SimulationStatus, reason?: string): void {
        this.endTime = Date.now();
        this.status = finalStatus;
        
        this.summary.endTime = this.endTime;
        this.summary.finalStatus = finalStatus;
        this.summary.duration = this.endTime - (this.startTime || this.endTime);

        this.append({
            type: StepType.SESSION_STOPPED,
            timestamp: this.endTime,
            stepNumber: this.entries.length,
            data: {
                reason: reason || 'normal_completion',
                status: finalStatus,
                duration: this.summary.duration,
                totalSteps: this.summary.totalSteps
            }
        });

        if (this.enableLogging) {
            this.logFinalSummary();
        }
    }

    private logFinalSummary(): void {
        console.log('\n=== Simulation Trace Summary ===');
        console.log(`Session: ${this.sessionId}`);
        console.log(`Profile: ${this.profileName}`);
        console.log(`Status: ${this.status}`);
        console.log(`Duration: ${(this.summary.duration / 1000).toFixed(2)}s`);
        console.log(`Total Steps: ${this.summary.totalSteps}`);
        console.log(`Motor Commands: ${this.summary.motorCommands}`);
        console.log(`Sensor Triggers: ${this.summary.sensorTriggers}`);
        console.log(`Conditions Met: ${this.summary.conditionsMet}`);
        console.log(`Errors: ${this.summary.errors}`);
        console.log(`Warnings: ${this.summary.warnings}`);
        console.log(`Constraint Violations: ${this.summary.constraintViolations}`);
        console.log('==============================\n');
    }

    public getEntries(filter?: { types?: StepType[]; startStep?: number; endStep?: number }): TraceEntry[] {
        let filtered = this.entries;

        if (filter?.types && filter.types.length > 0) {
            filtered = filtered.filter(entry => filter.types!.includes(entry.type));
        }

        if (filter?.startStep !== undefined) {
            filtered = filtered.filter(entry => entry.stepNumber >= filter.startStep!);
        }

        if (filter?.endStep !== undefined) {
            filtered = filtered.filter(entry => entry.stepNumber <= filter.endStep!);
        }

        return [...filtered];
    }

    public getSummary(): TraceSummary {
        return { ...this.summary };
    }

    public exportTrace(options?: TraceExportOptions): string {
        const exportOptions: TraceExportOptions = {
            includeData: true,
            format: 'json',
            ...options
        };

        let exportData: any;

        switch (exportOptions.format) {
            case 'minimal':
                exportData = {
                    sessionId: this.sessionId,
                    profile: this.profileName,
                    status: this.status,
                    summary: this.summary,
                    entries: this.getEntries(exportOptions).map(entry => ({
                        t: entry.type,
                        ts: entry.timestamp,
                        step: entry.stepNumber
                    }))
                };
                break;

            case 'debug':
                exportData = {
                    sessionId: this.sessionId,
                    profile: this.profileName,
                    status: this.status,
                    startTime: this.startTime,
                    endTime: this.endTime,
                    summary: this.summary,
                    entries: this.getEntries(exportOptions)
                };
                break;

            case 'json':
            default:
                exportData = {
                    metadata: {
                        sessionId: this.sessionId,
                        profile: this.profileName,
                        status: this.status,
                        startTime: this.startTime,
                        endTime: this.endTime
                    },
                    summary: this.summary,
                    entries: exportOptions.includeData 
                        ? this.getEntries(exportOptions)
                        : this.getEntries(exportOptions).map(({ data, ...rest }) => rest)
                };
                break;
        }

        return JSON.stringify(exportData, null, 2);
    }

    public getStateTimeline(): Array<{ step: number; timestamp: number; position: any; joints: any }> {
        return this.entries
            .filter(entry => entry.type === StepType.STATE_UPDATE && entry.data.newState)
            .map(entry => ({
                step: entry.stepNumber,
                timestamp: entry.timestamp,
                position: entry.data.newState.position,
                joints: entry.data.newState.joints
            }));
    }

    public getErrorLog(): Array<{ step: number; timestamp: number; error: string; type: string }> {
        return this.entries
            .filter(entry => entry.type === StepType.ERROR)
            .map(entry => ({
                step: entry.stepNumber,
                timestamp: entry.timestamp,
                error: entry.data.error || 'Unknown error',
                type: entry.data.errorType || 'unknown'
            }));
    }

    public getMotorCommandLog(): Array<{ step: number; timestamp: number; commands: any[] }> {
        const motorCommandEntries = this.entries.filter(entry => 
            entry.type === StepType.MOTOR_COMMAND || 
            (entry.type === StepType.STATE_UPDATE && entry.data.motorCommands)
        );

        return motorCommandEntries.map(entry => ({
            step: entry.stepNumber,
            timestamp: entry.timestamp,
            commands: entry.data.motorCommands || [entry.data]
        }));
    }

    public getSensorTriggerLog(): Array<{ step: number; timestamp: number; sensorId: string; value: any }> {
        const sensorEntries = this.entries.filter(entry => 
            entry.type === StepType.SENSOR_TRIGGER || 
            (entry.type === StepType.STATE_UPDATE && entry.data.sensorReadings)
        );

        const triggers: Array<{ step: number; timestamp: number; sensorId: string; value: any }> = [];

        sensorEntries.forEach(entry => {
            if (entry.data.sensorReadings) {
                entry.data.sensorReadings.forEach((reading: any) => {
                    triggers.push({
                        step: entry.stepNumber,
                        timestamp: entry.timestamp,
                        sensorId: reading.sensorId,
                        value: reading.value
                    });
                });
            } else if (entry.data.sensorId) {
                triggers.push({
                    step: entry.stepNumber,
                    timestamp: entry.timestamp,
                    sensorId: entry.data.sensorId,
                    value: entry.data.value
                });
            }
        });

        return triggers;
    }

    public clear(): void {
        this.entries = [];
        this.startTime = null;
        this.endTime = null;
        this.status = SimulationStatus.IDLE;
        
        this.summary = {
            totalSteps: 0,
            duration: 0,
            startTime: 0,
            endTime: 0,
            finalStatus: SimulationStatus.IDLE,
            errors: 0,
            warnings: 0,
            constraintViolations: 0,
            conditionsMet: 0,
            motorCommands: 0,
            sensorTriggers: 0
        };
    }

    public mergeTrace(otherTrace: ExecutionTrace): void {
        if (otherTrace.sessionId !== this.sessionId) {
            console.warn(`Merging traces from different sessions: ${this.sessionId} and ${otherTrace.sessionId}`);
        }

        const otherEntries = otherTrace.getEntries();
        this.entries.push(...otherEntries);
        
        const otherSummary = otherTrace.getSummary();
        
        this.summary.totalSteps += otherSummary.totalSteps;
        this.summary.errors += otherSummary.errors;
        this.summary.warnings += otherSummary.warnings;
        this.summary.constraintViolations += otherSummary.constraintViolations;
        this.summary.conditionsMet += otherSummary.conditionsMet;
        this.summary.motorCommands += otherSummary.motorCommands;
        this.summary.sensorTriggers += otherSummary.sensorTriggers;

        if (otherSummary.endTime > this.summary.endTime) {
            this.summary.endTime = otherSummary.endTime;
        }

        if (otherSummary.startTime < this.summary.startTime || this.summary.startTime === 0) {
            this.summary.startTime = otherSummary.startTime;
        }

        this.summary.duration = this.summary.endTime - this.summary.startTime;
    }

    public findStepByCondition(condition: (entry: TraceEntry) => boolean): TraceEntry | null {
        return this.entries.find(condition) || null;
    }

    public getStatistics(): TraceStatistics {
        const stateUpdates = this.entries.filter(e => e.type === StepType.STATE_UPDATE);
        const positions = stateUpdates.map(e => e.data.newState?.position).filter(Boolean);
        
        let totalDistance = 0;
        let maxSpeed = 0;
        
        for (let i = 1; i < positions.length; i++) {
            const prev = positions[i - 1];
            const curr = positions[i];
            
            if (prev && curr) {
                const dx = curr.x - prev.x;
                const dy = curr.y - prev.y;
                const dz = curr.z - prev.z;
                const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
                totalDistance += distance;
                
                const timeDelta = stateUpdates[i].timestamp - stateUpdates[i-1].timestamp;
                if (timeDelta > 0) {
                    const speed = distance / (timeDelta / 1000);
                    maxSpeed = Math.max(maxSpeed, speed);
                }
            }
        }

        return {
            totalDistance,
            maxSpeed,
            averageStepsPerSecond: this.summary.totalSteps / (this.summary.duration / 1000),
            errorRate: this.summary.errors / this.summary.totalSteps,
            efficiency: this.summary.conditionsMet / Math.max(1, this.summary.totalSteps)
        };
    }
}

export interface TraceStatistics {
    totalDistance: number;
    maxSpeed: number;
    averageStepsPerSecond: number;
    errorRate: number;
    efficiency: number;
}

export function createTraceAnalyzer(trace: ExecutionTrace): TraceAnalyzer {
    return new TraceAnalyzer(trace);
}

export class TraceAnalyzer {
    private trace: ExecutionTrace;

    constructor(trace: ExecutionTrace) {
        this.trace = trace;
    }

    public analyzePerformance(): PerformanceReport {
        const stats = this.trace.getStatistics();
        const summary = this.trace.getSummary();
        
        let performanceRating = 'good';
        if (stats.errorRate > 0.1) performanceRating = 'poor';
        else if (stats.errorRate > 0.05) performanceRating = 'fair';
        
        let efficiencyRating = 'good';
        if (stats.efficiency < 0.3) efficiencyRating = 'poor';
        else if (stats.efficiency < 0.6) efficiencyRating = 'fair';

        return {
            performanceRating,
            efficiencyRating,
            totalExecutionTime: summary.duration,
            stepsPerSecond: stats.averageStepsPerSecond,
            totalDistance: stats.totalDistance,
            maxSpeed: stats.maxSpeed,
            recommendations: this.generateRecommendations(stats, summary)
        };
    }

    private generateRecommendations(stats: TraceStatistics, summary: TraceSummary): string[] {
        const recommendations: string[] = [];

        if (stats.errorRate > 0.1) {
            recommendations.push('High error rate detected. Review command validation and constraints.');
        }

        if (stats.averageStepsPerSecond < 5) {
            recommendations.push('Low step rate. Consider optimizing task complexity.');
        }

        if (summary.constraintViolations > 0) {
            recommendations.push('Constraint violations occurred. Review joint limits and movement parameters.');
        }

        if (stats.efficiency < 0.3) {
            recommendations.push('Low condition completion rate. Check completion condition logic.');
        }

        return recommendations;
    }

    public exportAnalysis(format: 'json' | 'text'): string {
        const performance = this.analyzePerformance();
        const summary = this.trace.getSummary();
        const stats = this.trace.getStatistics();

        if (format === 'json') {
            return JSON.stringify({
                performance,
                summary,
                statistics: stats
            }, null, 2);
        } else {
            let text = '=== Trace Analysis Report ===\n\n';
            text += `Performance: ${performance.performanceRating}\n`;
            text += `Efficiency: ${performance.efficiencyRating}\n`;
            text += `Execution Time: ${(summary.duration / 1000).toFixed(2)}s\n`;
            text += `Steps/Second: ${stats.averageStepsPerSecond.toFixed(2)}\n`;
            text += `Total Distance: ${stats.totalDistance.toFixed(2)}m\n`;
            text += `Max Speed: ${stats.maxSpeed.toFixed(2)}m/s\n`;
            text += `Errors: ${summary.errors}\n`;
            text += `Constraint Violations: ${summary.constraintViolations}\n\n`;
            
            if (performance.recommendations.length > 0) {
                text += 'Recommendations:\n';
                performance.recommendations.forEach(rec => text += `- ${rec}\n`);
            }

            return text;
        }
    }
}

export interface PerformanceReport {
    performanceRating: 'poor' | 'fair' | 'good';
    efficiencyRating: 'poor' | 'fair' | 'good';
    totalExecutionTime: number;
    stepsPerSecond: number;
    totalDistance: number;
    maxSpeed: number;
    recommendations: string[];
}