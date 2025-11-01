/**
 * Configuration Management System for SynapseRobo SDK
 * 
 * Handles configuration loading, validation, and management across all SDK components
 * Supports environment variables, configuration files, and programmatic overrides
 */

import { existsSync, readFileSync, writeFileSync, mkdirSync } from 'fs';
import { join, dirname } from 'path';
import { logger } from './logger';
import { DEFAULT_CONFIG, FILE_PATHS } from '../constants';
import { SDKConfig, SolanaConfig, RobotConfig, AIConfig, LoggingConfig } from './types';

export class ConfigManager {
  private config: SDKConfig;
  private configPath: string;
  private environment: string;
  private validationRules: ValidationRules;

  constructor(configPath?: string) {
    this.configPath = configPath || FILE_PATHS.CONFIG;
    this.environment = process.env.NODE_ENV || 'development';
    this.config = this.loadDefaultConfig();
    this.validationRules = this.getValidationRules();
    
    this.ensureConfigDirectory();
  }

  /**
   * Initialize configuration with optional overrides
   */
  public initialize(overrides?: Partial<SDKConfig>): SDKConfig {
    try {
      // Load from file if exists
      if (existsSync(this.configPath)) {
        this.loadFromFile();
      }

      // Apply environment variables
      this.applyEnvironmentVariables();

      // Apply programmatic overrides
      if (overrides) {
        this.applyOverrides(overrides);
      }

      // Validate final configuration
      this.validateConfig();

      logger.info('Configuration initialized successfully', {
        environment: this.environment,
        configSource: existsSync(this.configPath) ? 'file' : 'default',
        overrides: overrides ? 'applied' : 'none'
      });

      return this.config;

    } catch (error) {
      logger.error('Failed to initialize configuration', error as Error);
      throw error;
    }
  }

  /**
   * Get current configuration
   */
  public getConfig(): SDKConfig {
    return { ...this.config };
  }

  /**
   * Update configuration with new values
   */
  public updateConfig(updates: Partial<SDKConfig>): void {
    try {
      this.applyOverrides(updates);
      this.validateConfig();
      
      logger.info('Configuration updated successfully', {
        updatedSections: Object.keys(updates)
      });
    } catch (error) {
      logger.error('Failed to update configuration', error as Error);
      throw error;
    }
  }

  /**
   * Save current configuration to file
   */
  public saveToFile(path?: string): void {
    try {
      const savePath = path || this.configPath;
      const configDir = dirname(savePath);
      
      if (!existsSync(configDir)) {
        mkdirSync(configDir, { recursive: true });
      }

      const configData = {
        $schema: './synapse-config.schema.json',
        environment: this.environment,
        timestamp: new Date().toISOString(),
        version: '1.0.0',
        ...this.config
      };

      writeFileSync(savePath, JSON.stringify(configData, null, 2), 'utf8');
      
      logger.info('Configuration saved to file', { path: savePath });
    } catch (error) {
      logger.error('Failed to save configuration to file', error as Error);
      throw error;
    }
  }

  /**
   * Reset configuration to defaults
   */
  public resetToDefaults(): void {
    this.config = this.loadDefaultConfig();
    logger.info('Configuration reset to defaults');
  }

  /**
   * Validate configuration against schema
   */
  public validateConfig(): ValidationResult {
    const errors: ValidationError[] = [];
    const warnings: ValidationWarning[] = [];

    // Validate Solana configuration
    const solanaErrors = this.validateSolanaConfig(this.config.solana);
    errors.push(...solanaErrors);

    // Validate Robot configuration
    const robotErrors = this.validateRobotConfig(this.config.robot);
    errors.push(...robotErrors);

    // Validate AI configuration
    const aiErrors = this.validateAIConfig(this.config.ai);
    errors.push(...aiErrors);

    // Validate Logging configuration
    const loggingErrors = this.validateLoggingConfig(this.config.logging);
    errors.push(...loggingErrors);

    // Check for deprecated settings
    const deprecationWarnings = this.checkDeprecatedSettings();
    warnings.push(...deprecationWarnings);

    // Performance and security warnings
    const performanceWarnings = this.checkPerformanceSettings();
    warnings.push(...performanceWarnings);

    const validationResult: ValidationResult = {
      valid: errors.length === 0,
      errors,
      warnings,
      timestamp: new Date()
    };

    if (!validationResult.valid) {
      logger.warn('Configuration validation failed', {
        errorCount: errors.length,
        warningCount: warnings.length,
        errors: errors.map(e => e.message)
      });
    } else if (warnings.length > 0) {
      logger.info('Configuration validated with warnings', {
        warningCount: warnings.length,
        warnings: warnings.map(w => w.message)
      });
    } else {
      logger.debug('Configuration validation passed');
    }

    return validationResult;
  }

  /**
   * Get configuration for specific environment
   */
  public getEnvironmentConfig(environment: string): Partial<SDKConfig> {
    const environmentConfigs: { [key: string]: Partial<SDKConfig> } = {
      development: {
        logging: {
          level: 3, // DEBUG
          format: 'text',
          enableTelemetry: false
        },
        robot: {
          simulation: true,
          maxVelocity: 0.3
        }
      },
      production: {
        logging: {
          level: 1, // WARN
          format: 'json',
          enableTelemetry: true
        },
        robot: {
          simulation: false,
          maxVelocity: 0.5
        },
        solana: {
          network: 'mainnet',
          commitment: 'finalized'
        }
      },
      testing: {
        logging: {
          level: 4, // TRACE
          format: 'json',
          enableTelemetry: false
        },
        robot: {
          simulation: true,
          maxVelocity: 0.2
        }
      }
    };

    return environmentConfigs[environment] || {};
  }

  /**
   * Create configuration schema for validation
   */
  private getValidationRules(): ValidationRules {
    return {
      solana: {
        network: {
          required: true,
          type: 'string',
          allowedValues: ['mainnet', 'devnet', 'testnet', 'localnet']
        },
        commitment: {
          required: true,
          type: 'string',
          allowedValues: ['processed', 'confirmed', 'finalized']
        },
        maxRetries: {
          required: true,
          type: 'number',
          min: 1,
          max: 10
        },
        retryDelay: {
          required: true,
          type: 'number',
          min: 100,
          max: 10000
        }
      },
      robot: {
        simulation: {
          required: true,
          type: 'boolean'
        },
        timeout: {
          required: true,
          type: 'number',
          min: 1000,
          max: 300000
        },
        maxVelocity: {
          required: true,
          type: 'number',
          min: 0.1,
          max: 1.0
        },
        safetyEnabled: {
          required: true,
          type: 'boolean'
        }
      },
      ai: {
        model: {
          required: true,
          type: 'string'
        },
        temperature: {
          required: false,
          type: 'number',
          min: 0,
          max: 2
        },
        maxTokens: {
          required: false,
          type: 'number',
          min: 1,
          max: 10000
        },
        timeout: {
          required: true,
          type: 'number',
          min: 1000,
          max: 300000
        }
      },
      logging: {
        level: {
          required: true,
          type: 'number',
          min: 0,
          max: 4
        },
        format: {
          required: true,
          type: 'string',
          allowedValues: ['json', 'text']
        },
        enableTelemetry: {
          required: true,
          type: 'boolean'
        }
      }
    };
  }

  private loadDefaultConfig(): SDKConfig {
    const environmentConfig = this.getEnvironmentConfig(this.environment);
    
    return {
      solana: { ...DEFAULT_CONFIG.solana, ...environmentConfig.solana },
      robot: { ...DEFAULT_CONFIG.robot, ...environmentConfig.robot },
      ai: { ...DEFAULT_CONFIG.ai, ...environmentConfig.ai },
      logging: { ...DEFAULT_CONFIG.logging, ...environmentConfig.logging }
    };
  }

  private loadFromFile(): void {
    try {
      const fileContent = readFileSync(this.configPath, 'utf8');
      const fileConfig = JSON.parse(fileContent);
      
      // Merge file config with defaults
      this.config = {
        solana: { ...this.config.solana, ...fileConfig.solana },
        robot: { ...this.config.robot, ...fileConfig.robot },
        ai: { ...this.config.ai, ...fileConfig.ai },
        logging: { ...this.config.logging, ...fileConfig.logging }
      };

      logger.debug('Configuration loaded from file', { path: this.configPath });
    } catch (error) {
      logger.warn('Failed to load configuration from file, using defaults', {
        path: this.configPath,
        error: (error as Error).message
      });
    }
  }

  private applyEnvironmentVariables(): void {
    const envConfig: Partial<SDKConfig> = {};

    // Solana environment variables
    if (process.env.SOLANA_NETWORK) {
      envConfig.solana = {
        ...envConfig.solana,
        network: process.env.SOLANA_NETWORK
      };
    }

    if (process.env.SOLANA_COMMITMENT) {
      envConfig.solana = {
        ...envConfig.solana,
        commitment: process.env.SOLANA_COMMITMENT as any
      };
    }

    if (process.env.SOLANA_MAX_RETRIES) {
      envConfig.solana = {
        ...envConfig.solana,
        maxRetries: parseInt(process.env.SOLANA_MAX_RETRIES)
      };
    }

    // Robot environment variables
    if (process.env.ROBOT_SIMULATION) {
      envConfig.robot = {
        ...envConfig.robot,
        simulation: process.env.ROBOT_SIMULATION === 'true'
      };
    }

    if (process.env.ROBOT_MAX_VELOCITY) {
      envConfig.robot = {
        ...envConfig.robot,
        maxVelocity: parseFloat(process.env.ROBOT_MAX_VELOCITY)
      };
    }

    // AI environment variables
    if (process.env.AI_MODEL) {
      envConfig.ai = {
        ...envConfig.ai,
        model: process.env.AI_MODEL as any
      };
    }

    if (process.env.AI_TEMPERATURE) {
      envConfig.ai = {
        ...envConfig.ai,
        temperature: parseFloat(process.env.AI_TEMPERATURE)
      };
    }

    // Logging environment variables
    if (process.env.LOG_LEVEL) {
      envConfig.logging = {
        ...envConfig.logging,
        level: parseInt(process.env.LOG_LEVEL)
      };
    }

    if (process.env.LOG_FORMAT) {
      envConfig.logging = {
        ...envConfig.logging,
        format: process.env.LOG_FORMAT as any
      };
    }

    if (process.env.ENABLE_TELEMETRY) {
      envConfig.logging = {
        ...envConfig.logging,
        enableTelemetry: process.env.ENABLE_TELEMETRY === 'true'
      };
    }

    this.applyOverrides(envConfig);
  }

  private applyOverrides(overrides: Partial<SDKConfig>): void {
    if (overrides.solana) {
      this.config.solana = { ...this.config.solana, ...overrides.solana };
    }
    if (overrides.robot) {
      this.config.robot = { ...this.config.robot, ...overrides.robot };
    }
    if (overrides.ai) {
      this.config.ai = { ...this.config.ai, ...overrides.ai };
    }
    if (overrides.logging) {
      this.config.logging = { ...this.config.logging, ...overrides.logging };
    }
  }

  private validateSolanaConfig(config: SolanaConfig): ValidationError[] {
    const errors: ValidationError[] = [];
    const rules = this.validationRules.solana;

    if (!config.network) {
      errors.push({
        section: 'solana',
        field: 'network',
        message: 'Solana network is required'
      });
    } else if (!rules.network.allowedValues?.includes(config.network)) {
      errors.push({
        section: 'solana',
        field: 'network',
        message: `Invalid Solana network: ${config.network}. Allowed values: ${rules.network.allowedValues.join(', ')}`
      });
    }

    if (config.maxRetries < rules.maxRetries.min! || config.maxRetries > rules.maxRetries.max!) {
      errors.push({
        section: 'solana',
        field: 'maxRetries',
        message: `Solana maxRetries must be between ${rules.maxRetries.min} and ${rules.maxRetries.max}`
      });
    }

    return errors;
  }

  private validateRobotConfig(config: RobotConfig): ValidationError[] {
    const errors: ValidationError[] = [];
    const rules = this.validationRules.robot;

    if (config.maxVelocity < rules.maxVelocity.min! || config.maxVelocity > rules.maxVelocity.max!) {
      errors.push({
        section: 'robot',
        field: 'maxVelocity',
        message: `Robot maxVelocity must be between ${rules.maxVelocity.min} and ${rules.maxVelocity.max}`
      });
    }

    if (config.timeout < rules.timeout.min! || config.timeout > rules.timeout.max!) {
      errors.push({
        section: 'robot',
        field: 'timeout',
        message: `Robot timeout must be between ${rules.timeout.min} and ${rules.timeout.max} ms`
      });
    }

    return errors;
  }

  private validateAIConfig(config: AIConfig): ValidationError[] {
    const errors: ValidationError[] = [];
    const rules = this.validationRules.ai;

    if (!config.model) {
      errors.push({
        section: 'ai',
        field: 'model',
        message: 'AI model is required'
      });
    }

    if (config.temperature !== undefined && (config.temperature < rules.temperature.min! || config.temperature > rules.temperature.max!)) {
      errors.push({
        section: 'ai',
        field: 'temperature',
        message: `AI temperature must be between ${rules.temperature.min} and ${rules.temperature.max}`
      });
    }

    if (config.timeout < rules.timeout.min! || config.timeout > rules.timeout.max!) {
      errors.push({
        section: 'ai',
        field: 'timeout',
        message: `AI timeout must be between ${rules.timeout.min} and ${rules.timeout.max} ms`
      });
    }

    return errors;
  }

  private validateLoggingConfig(config: LoggingConfig): ValidationError[] {
    const errors: ValidationError[] = [];
    const rules = this.validationRules.logging;

    if (config.level < rules.level.min! || config.level > rules.level.max!) {
      errors.push({
        section: 'logging',
        field: 'level',
        message: `Logging level must be between ${rules.level.min} and ${rules.level.max}`
      });
    }

    if (!rules.format.allowedValues?.includes(config.format)) {
      errors.push({
        section: 'logging',
        field: 'format',
        message: `Invalid logging format: ${config.format}. Allowed values: ${rules.format.allowedValues.join(', ')}`
      });
    }

    return errors;
  }

  private checkDeprecatedSettings(): ValidationWarning[] {
    const warnings: ValidationWarning[] = [];

    // Check for deprecated settings that might be in use
    if (this.config.solana.network === 'testnet') {
      warnings.push({
        section: 'solana',
        field: 'network',
        message: 'Testnet is deprecated and will be removed in future versions. Use devnet instead.',
        severity: 'medium'
      });
    }

    return warnings;
  }

  private checkPerformanceSettings(): ValidationWarning[] {
    const warnings: ValidationWarning[] = [];

    // Performance warnings
    if (this.config.robot.maxVelocity > 0.8) {
      warnings.push({
        section: 'robot',
        field: 'maxVelocity',
        message: 'High maximum velocity may impact stability and safety',
        severity: 'low'
      });
    }

    if (this.config.ai.timeout > 120000) {
      warnings.push({
        section: 'ai',
        field: 'timeout',
        message: 'Long AI timeout may impact responsiveness',
        severity: 'low'
      });
    }

    return warnings;
  }

  private ensureConfigDirectory(): void {
    const configDir = dirname(this.configPath);
    if (!existsSync(configDir)) {
      try {
        mkdirSync(configDir, { recursive: true });
        logger.debug('Configuration directory created', { path: configDir });
      } catch (error) {
        logger.warn('Failed to create configuration directory', {
          path: configDir,
          error: (error as Error).message
        });
      }
    }
  }
}

// Type definitions for configuration validation
export interface ValidationRules {
  [section: string]: {
    [field: string]: {
      required: boolean;
      type: string;
      min?: number;
      max?: number;
      allowedValues?: string[];
    };
  };
}

export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
  warnings: ValidationWarning[];
  timestamp: Date;
}

export interface ValidationError {
  section: string;
  field: string;
  message: string;
}

export interface ValidationWarning {
  section: string;
  field: string;
  message: string;
  severity: 'low' | 'medium' | 'high';
}

// Export singleton instance for convenience
export const configManager = new ConfigManager();