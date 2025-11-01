/**
 * @file aiEngine.ts
 * @description Core AI Engine for SynapseRobo SDK
 * Handles LLM inference, action plan generation, safety filtering, streaming responses,
 * sensor integration, and decision graph management with offline simulation support.
 */

import { Subject, Observable, from, of, throwError, timer, interval } from 'rxjs';
import { map, catchError, retry, timeout, switchMap, takeUntil, filter, tap, mergeMap } from 'rxjs/operators';
import { v4 as uuidv4 } from 'uuid';
import { logger } from '@utils/logger';
import { AI_MODELS, COMMAND_TYPES, SAFETY_RULES, PROMPT_TEMPLATES } from '../constants';
import { ActionPlan, AIAction, AIObservation, SensorData, DecisionNode, SafetyViolation, AIConfig, ModelResponse } from '@utils/types';
import { HardwareAdapter } from '@adapters/hardwareAdapter';
import { SimulationAdapter } from '@adapters/simulationAdapter';

interface InferenceProvider {
  generate(prompt: string, config: Partial<AIConfig>): Promise<string>;
  stream(prompt: string, config: Partial<AIConfig>): Observable<string>;
}

interface ActionValidationResult {
  valid: boolean;
  violations: SafetyViolation[];
  adjustedAction?: AIAction;
}

interface DecisionContext {
  robotState: any;
  sensorData: SensorData;
  taskGoal: string;
  previousActions: AIAction[];
  timestamp: number;
}

export class AIEngine {
  private config: AIConfig;
  private provider: InferenceProvider | null = null;
  private decisionGraph: Map<string, DecisionNode> = new Map();
  private actionHistory: AIAction[] = [];
  private observationBuffer: AIObservation[] = [];
  private safetyRules: SAFETY_RULES[] = [];
  private promptTemplates: Map<string, string> = new Map();
  private streamingSubject = new Subject<string>();
  private stopStreaming = new Subject<void>();
  private isInitialized = false;
  private modelCache: Map<string, any> = new Map();
  private hardwareAdapter?: HardwareAdapter;
  private simulationAdapter?: SimulationAdapter;
  private inferenceQueue: AIAction[] = [];
  private processing = false;

  /**
   * Creates a new AIEngine instance
   * @param config - AI engine configuration
   */
  constructor(config: Partial<AIConfig> = {}) {
    this.config = {
      model: config.model || AI_MODELS.LOCAL_LLM,
      temperature: config.temperature ?? 0.7,
      maxTokens: config.maxTokens ?? 1000,
      timeout: config.timeout ?? 30000,
      safetyEnabled: config.safetyEnabled ?? true,
      streaming: config.streaming ?? false,
      offlineMode: config.offlineMode ?? false,
      simulationMode: config.simulationMode ?? false,
      maxHistory: config.maxHistory ?? 50,
      retryAttempts: config.retryAttempts ?? 3,
      cacheEnabled: config.cacheEnabled ?? true,
    };

    this.initializeSafetyRules();
    this.initializePromptTemplates();
    this.setupDecisionGraph();
  }

  /**
   * Initializes the AI engine and providers
   */
  public async initialize(): Promise<void> {
    if (this.isInitialized) {
      logger.warn('AI Engine already initialized');
      return;
    }

    logger.info('Initializing AI Engine...', { model: this.config.model });

    try {
      if (this.config.simulationMode || this.config.offlineMode) {
        this.provider = this.createSimulationProvider();
        logger.info('Using simulation inference provider');
      } else {
        await this.initializeRealProvider();
      }

      this.startBackgroundProcesses();
      this.isInitialized = true;
      logger.info('AI Engine initialized successfully');
    } catch (error) {
      logger.error('Failed to initialize AI Engine', error as Error);
      throw error;
    }
  }

  /**
   * Generates a complete action plan based on the prompt
   * @param prompt - Task description or user instruction
   * @param context - Additional context including robot state and sensors
   * @returns Action plan with estimated duration and complexity
   */
  public async generateActionPlan(
    prompt: string,
    context?: Partial<DecisionContext>
  ): Promise<ActionPlan> {
    this.validateInitialization();

    const requestId = uuidv4();
    const startTime = Date.now();
    
    logger.info('Generating action plan', { requestId, promptLength: prompt.length });

    try {
      const enrichedPrompt = this.enrichPrompt(prompt, context);
      const rawResponse = await this.inferenceWithRetry(enrichedPrompt);
      const parsedPlan = this.parseActionPlan(rawResponse, requestId);
      
      const validatedPlan = await this.validateActionPlan(parsedPlan);
      const optimizedPlan = this.optimizeActionPlan(validatedPlan);
      
      const actionPlan: ActionPlan = {
        id: requestId,
        actions: optimizedPlan,
        estimatedDuration: this.estimatePlanDuration(optimizedPlan),
        complexity: this.calculateComplexity(optimizedPlan),
        confidence: this.calculateConfidence(rawResponse, optimizedPlan),
        timestamp: Date.now(),
        model: this.config.model,
        requestId,
      };

      this.cacheActionPlan(prompt, actionPlan);
      logger.info('Action plan generated successfully', {
        requestId,
        actionCount: actionPlan.actions.length,
        duration: Date.now() - startTime,
      });

      return actionPlan;
    } catch (error) {
      logger.error('Failed to generate action plan', { requestId, error: error as Error });
      throw error;
    }
  }

  /**
   * Streams action plan generation in real-time
   * @param prompt - Task prompt
   * @param context - Decision context
   * @returns Observable emitting partial responses
   */
  public streamActionPlan(
    prompt: string,
    context?: Partial<DecisionContext>
  ): Observable<string> {
    this.validateInitialization();
    this.stopStreaming.next();

    const enrichedPrompt = this.enrichPrompt(prompt, context);
    const stream$ = this.provider!.stream(enrichedPrompt, this.config);

    return stream$.pipe(
      tap(chunk => this.streamingSubject.next(chunk)),
      takeUntil(this.stopStreaming),
      catchError(error => {
        logger.error('Streaming error', error);
        return throwError(() => error);
      })
    );
  }

  /**
   * Updates AI with new observations from environment
   * @param observations - Sensor observations and outcomes
   */
  public updateWithObservations(observations: AIObservation[]): void {
    if (!this.config.safetyEnabled) return;

    this.observationBuffer.push(...observations);
    
    // Keep buffer size manageable
    if (this.observationBuffer.length > this.config.maxHistory!) {
      this.observationBuffer = this.observationBuffer.slice(-this.config.maxHistory!);
    }

    this.updateDecisionGraph(observations);
    logger.debug('Updated AI with observations', { count: observations.length });
  }

  /**
   * Requests AI to replan based on failure or environment change
   * @param failedAction - Action that failed
   * @param error - Error information
   * @returns New action plan or null
   */
  public async requestReplan(
    failedAction: AIAction,
    error: string | Error
  ): Promise<ActionPlan | null> {
    const replanPrompt = this.buildReplanPrompt(failedAction, error);
    return this.generateActionPlan(replanPrompt);
  }

  /**
   * Validates a single action against safety rules
   * @param action - Action to validate
   * @param context - Current robot context
   * @returns Validation result with possible adjustments
   */
  public validateAction(
    action: AIAction,
    context?: DecisionContext
  ): ActionValidationResult {
    const violations: SafetyViolation[] = [];
    let adjustedAction = { ...action };

    for (const rule of this.safetyRules) {
      const result = this.applySafetyRule(rule, adjustedAction, context);
      if (!result.valid) {
        violations.push(result.violation!);
        
        if (result.adjustment) {
          adjustedAction = { ...adjustedAction, ...result.adjustment };
        }
      }
    }

    return {
      valid: violations.length === 0,
      violations,
      adjustedAction: violations.length > 0 ? adjustedAction : undefined,
    };
  }

  /**
   * Sets hardware adapter for sensor integration
   * @param adapter - Hardware adapter instance
   */
  public setHardwareAdapter(adapter: HardwareAdapter): void {
    this.hardwareAdapter = adapter;
    logger.info('Hardware adapter connected to AI Engine');
  }

  /**
   * Sets simulation adapter for offline testing
   * @param adapter - Simulation adapter instance
   */
  public setSimulationAdapter(adapter: SimulationAdapter): void {
    this.simulationAdapter = adapter;
    logger.info('Simulation adapter connected to AI Engine');
  }

  /**
   * Stops all streaming operations
   */
  public stopStreamingResponse(): void {
    this.stopStreaming.next();
  }

  /**
   * Gets current AI engine status
   */
  public getStatus(): {
    initialized: boolean;
    model: string;
    offline: boolean;
    simulation: boolean;
    queueLength: number;
  } {
    return {
      initialized: this.isInitialized,
      model: this.config.model,
      offline: this.config.offlineMode,
      simulation: this.config.simulationMode,
      queueLength: this.inferenceQueue.length,
    };
  }

  /**
   * Clears cache and history
   */
  public clearCache(): void {
    this.modelCache.clear();
    this.actionHistory = [];
    this.observationBuffer = [];
    logger.info('AI Engine cache cleared');
  }

  private validateInitialization(): void {
    if (!this.isInitialized) {
      throw new Error('AI Engine not initialized. Call initialize() first.');
    }
  }

  private initializeSafetyRules(): void {
    this.safetyRules = [
      {
        id: 'speed_limit',
        description: 'Maximum movement speed',
        check: (action, context) => {
          if (action.type === COMMAND_TYPES.MOVE && action.parameters?.speed) {
            const maxSpeed = context?.robotState?.maxVelocity || 0.3;
            return action.parameters.speed <= maxSpeed;
          }
          return true;
        },
      },
      {
        id: 'force_limit',
        description: 'Maximum grip force',
        check: (action, context) => {
          if (action.type === COMMAND_TYPES.GRIP && action.parameters?.force) {
            return action.parameters.force <= 20; // Newtons
          }
          return true;
        },
      },
      {
        id: 'height_limit',
        description: 'Step height safety',
        check: (action, context) => {
          if (action.type === COMMAND_TYPES.WALK && action.parameters?.stepHeight) {
            const maxStep = context?.robotState?.maxStepHeight || 0.15;
            return action.parameters.stepHeight <= maxStep;
          }
          return true;
        },
      },
      {
        id: 'balance_check',
        description: 'Balance maintenance',
        check: (action, context) => {
          if (action.type === COMMAND_TYPES.MOVE && context?.robotState?.balanceEnabled) {
            return context.sensorData?.imu?.stability || 0 > 0.8;
          }
          return true;
        },
      },
    ];
  }

  private initializePromptTemplates(): void {
    this.promptTemplates.set('exploration', PROMPT_TEMPLATES.EXPLORATION);
    this.promptTemplates.set('retrieval', PROMPT_TEMPLATES.RETRIEVAL);
    this.promptTemplates.set('interactive', PROMPT_TEMPLATES.INTERACTIVE);
    this.promptTemplates.set('default', PROMPT_TEMPLATES.DEFAULT);
  }

  private setupDecisionGraph(): void {
    // Root decision node
    const root: DecisionNode = {
      id: 'root',
      type: 'task_analysis',
      children: ['goal_decomposition', 'risk_assessment'],
      weight: 1.0,
    };

    const decomposition: DecisionNode = {
      id: 'goal_decomposition',
      type: 'decomposition',
      children: ['action_sequence'],
      weight: 0.8,
    };

    const risk: DecisionNode = {
      id: 'risk_assessment',
      type: 'safety',
      children: ['action_sequence'],
      weight: 0.9,
    };

    const sequence: DecisionNode = {
      id: 'action_sequence',
      type: 'planning',
      children: [],
      weight: 1.0,
    };

    this.decisionGraph.set('root', root);
    this.decisionGraph.set('goal_decomposition', decomposition);
    this.decisionGraph.set('risk_assessment', risk);
    this.decisionGraph.set('action_sequence', sequence);
  }

  private createSimulationProvider(): InferenceProvider {
    return {
      generate: async (prompt: string) => {
        await this.delay(500 + Math.random() * 1000);
        return this.generateMockResponse(prompt);
      },
      stream: (prompt: string) => {
        return interval(100).pipe(
          map(i => this.generateMockChunk(prompt, i)),
          takeUntil(timer(3000)),
          takeUntil(this.stopStreaming)
        );
      },
    };
  }

  private async initializeRealProvider(): Promise<void> {
    // In real implementation, this would connect to actual LLM providers
    // For now, use simulation provider as fallback
    this.provider = this.createSimulationProvider();
  }

  private startBackgroundProcesses(): void {
    // Process inference queue
    interval(100).pipe(
      filter(() => this.inferenceQueue.length > 0 && !this.processing),
      tap(() => this.processInferenceQueue())
    ).subscribe();
  }

  private enrichPrompt(prompt: string, context?: Partial<DecisionContext>): string {
    const template = this.getPromptTemplate(prompt) || this.promptTemplates.get('default')!;
    const robotState = context?.robotState ? JSON.stringify(context.robotState) : '{}';
    const sensors = context?.sensorData ? JSON.stringify(context.sensorData) : '{}';

    return template
      .replace('{{PROMPT}}', prompt)
      .replace('{{ROBOT_STATE}}', robotState)
      .replace('{{SENSORS}}', sensors)
      .replace('{{HISTORY}}', JSON.stringify(this.actionHistory.slice(-5)));
  }

  private getPromptTemplate(prompt: string): string | undefined {
    const keywords = prompt.toLowerCase();
    if (keywords.includes('explore') || keywords.includes('map')) {
      return this.promptTemplates.get('exploration');
    }
    if (keywords.includes('pick') || keywords.includes('retrieve')) {
      return this.promptTemplates.get('retrieval');
    }
    if (keywords.includes('dynamic') || keywords.includes('adapt')) {
      return this.promptTemplates.get('interactive');
    }
    return undefined;
  }

  private async inferenceWithRetry(prompt: string): Promise<string> {
    const operation = async () => {
      if (!this.provider) throw new Error('No inference provider');
      return this.provider.generate(prompt, this.config);
    };

    return from(operation()).pipe(
      retry(this.config.retryAttempts!),
      timeout(this.config.timeout!),
      catchError(error => {
        logger.error('Inference failed after retries', error);
        return throwError(() => error);
      })
    ).toPromise() as Promise<string>;
  }

  private parseActionPlan(response: string, requestId: string): AIAction[] {
    try {
      const jsonMatch = response.match(/```json\n([\s\S]*?)\n```/);
      if (jsonMatch) {
        const actions = JSON.parse(jsonMatch[1]);
        return actions.map((a: any, index: number) => ({
          ...a,
          id: `${requestId}-action-${index}`,
          timestamp: Date.now(),
        }));
      }

      // Fallback parsing
      return this.parseNaturalLanguageActions(response, requestId);
    } catch (error) {
      logger.warn('JSON parsing failed, using fallback', error);
      return this.parseNaturalLanguageActions(response, requestId);
    }
  }

  private parseNaturalLanguageActions(text: string, requestId: string): AIAction[] {
    const lines = text.split('\n').filter(l => l.trim());
    const actions: AIAction[] = [];
    let currentAction: Partial<AIAction> = {};

    for (const line of lines) {
      if (line.match(/^\d+\./)) {
        if (currentAction.type) {
          actions.push(currentAction as AIAction);
        }
        currentAction = {
          id: `${requestId}-action-${actions.length}`,
          description: line.replace(/^\d+\.\s*/, ''),
          timestamp: Date.now(),
        };
      } else if (line.includes(':')) {
        const [key, value] = line.split(':').map(s => s.trim());
        if (key.toLowerCase() === 'type') {
          currentAction.type = value.toUpperCase().replace(/\s/g, '_');
        } else if (key.toLowerCase() === 'duration') {
          currentAction.duration = parseFloat(value);
        }
      }
    }

    if (currentAction.type) {
      actions.push(currentAction as AIAction);
    }

    return actions;
  }

  private async validateActionPlan(plan: AIAction[]): Promise<AIAction[]> {
    const validated: AIAction[] = [];
    
    for (const action of plan) {
      const context = this.buildValidationContext();
      const validation = this.validateAction(action, context);
      
      if (validation.valid) {
        validated.push(validation.adjustedAction || action);
      } else {
        logger.warn('Action filtered by safety rules', {
          action: action.description,
          violations: validation.violations.map(v => v.ruleId),
        });
      }
    }

    return validated;
  }

  private buildValidationContext(): DecisionContext {
    return {
      robotState: this.hardwareAdapter?.getState() || this.simulationAdapter?.getState() || {},
      sensorData: this.getLatestSensorData(),
      taskGoal: 'current_task',
      previousActions: this.actionHistory.slice(-10),
      timestamp: Date.now(),
    };
  }

  private getLatestSensorData(): SensorData {
    return {
      camera: this.simulationAdapter?.getCameraData() || [],
      lidar: this.simulationAdapter?.getLidarData() || [],
      imu: this.simulationAdapter?.getIMUData() || { stability: 1.0 },
      timestamp: Date.now(),
    };
  }

  private optimizeActionPlan(plan: AIAction[]): AIAction[] {
    // Remove duplicate consecutive actions
    const optimized: AIAction[] = [];
    let lastAction: AIAction | null = null;

    for (const action of plan) {
      if (!lastAction || 
          lastAction.type !== action.type || 
          JSON.stringify(lastAction.parameters) !== JSON.stringify(action.parameters)) {
        optimized.push(action);
        lastAction = action;
      }
    }

    return optimized;
  }

  private estimatePlanDuration(plan: AIAction[]): number {
    return plan.reduce((sum, action) => sum + (action.duration || 2), 0);
  }

  private calculateComplexity(plan: AIAction[]): number {
    const weights = {
      [COMMAND_TYPES.MOVE]: 1,
      [COMMAND_TYPES.TURN]: 1,
      [COMMAND_TYPES.GRIP]: 3,
      [COMMAND_TYPES.LIFT]: 2,
      [COMMAND_TYPES.EXAMINE]: 1,
    };

    return plan.reduce((sum, action) => 
      sum + (weights[action.type as keyof typeof weights] || 1), 0
    ) / plan.length;
  }

  private calculateConfidence(response: string, plan: AIAction[]): number {
    const responseLength = response.length;
    const actionCount = plan.length;
    const jsonScore = response.includes('```json') ? 1.0 : 0.5;
    
    return Math.min(1.0, (responseLength / 1000) * actionCount * jsonScore);
  }

  private applySafetyRule(
    rule: SAFETY_RULES,
    action: AIAction,
    context?: DecisionContext
  ): { valid: boolean; violation?: SafetyViolation; adjustment?: any } {
    const valid = rule.check(action, context);
    
    if (valid) {
      return { valid: true };
    }

    return {
      valid: false,
      violation: {
        ruleId: rule.id,
        severity: 'high',
        message: rule.description,
        timestamp: Date.now(),
      },
      adjustment: this.getRuleAdjustment(rule.id, action),
    };
  }

  private getRuleAdjustment(ruleId: string, action: AIAction): any {
    switch (ruleId) {
      case 'speed_limit':
        return { parameters: { ...action.parameters, speed: 0.2 } };
      case 'force_limit':
        return { parameters: { ...action.parameters, force: 15 } };
      case 'height_limit':
        return { parameters: { ...action.parameters, stepHeight: 0.1 } };
      default:
        return undefined;
    }
  }

  private updateDecisionGraph(observations: AIObservation[]): void {
    // Update node weights based on outcomes
    observations.forEach(obs => {
      if (obs.success) {
        this.updateNodeWeight(obs.actionType, 0.1);
      } else {
        this.updateNodeWeight(obs.actionType, -0.05);
      }
    });
  }

  private updateNodeWeight(actionType: string, delta: number): void {
    for (const node of this.decisionGraph.values()) {
      if (node.type === 'planning' && node.children.length === 0) {
        node.weight = Math.max(0.1, Math.min(1.0, (node.weight || 1.0) + delta));
      }
    }
  }

  private buildReplanPrompt(failedAction: AIAction, error: string | Error): string {
    const errorMsg = error instanceof Error ? error.message : error;
    return `
Previous action failed: ${failedAction.description}
Error: ${errorMsg}
Current robot state: ${JSON.stringify(this.hardwareAdapter?.getState() || {})}
Please generate an alternative action plan to achieve the same goal.
Focus on avoiding the specific failure cause.
    `.trim();
  }

  private cacheActionPlan(prompt: string, plan: ActionPlan): void {
    if (!this.config.cacheEnabled) return;

    const hash = this.hashString(prompt);
    this.modelCache.set(hash, {
      plan,
      timestamp: Date.now(),
      prompt,
    });
  }

  private hashString(str: string): string {
    let hash = 0;
    for (let i = 0; i < str.length; i++) {
      const char = str.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash;
    }
    return hash.toString();
  }

  private generateMockResponse(prompt: string): string {
    const mockPlans = [
      `Here is a detailed action plan in JSON format:

\`\`\`json
[
  {
    "type": "MOVE_FORWARD",
    "description": "Move forward 1 meter",
    "parameters": {"distance": 1.0, "speed": 0.2},
    "duration": 5.0
  },
  {
    "type": "TURN_RIGHT",
    "description": "Turn 90 degrees right",
    "parameters": {"angle": 90},
    "duration": 2.0
  },
  {
    "type": "EXAMINE_OBJECT",
    "description": "Scan object with camera",
    "parameters": {"duration": 3},
    "duration": 3.0
  }
]
\`\`\`

This plan will systematically explore the area while maintaining safety.`
    ];

    return mockPlans[0];
  }

  private generateMockChunk(prompt: string, index: number): string {
    const chunks = [
      "Analyzing task requirements...\n",
      "Decomposing goal into atomic actions...\n",
      "```json\n[\n",
      '  {"type": "MOVE_FORWARD", "description": "Move forward", "parameters": {"distance": 1.0}}\n',
      ']```',
    ];
    
    return index < chunks.length ? chunks[index] : '';
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  private async processInferenceQueue(): Promise<void> {
    if (this.processing || this.inferenceQueue.length === 0) return;
    
    this.processing = true;
    const action = this.inferenceQueue.shift()!;
    
    try {
      // Process action inference
      await this.delay(100);
    } catch (error) {
      logger.error('Queue processing error', error as Error);
    } finally {
      this.processing = false;
    }
  }
}