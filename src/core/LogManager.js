import { ServiceContainer } from './ServiceContainer.js';

/**
 * LogManager - Centralized logging system with severity levels
 * Supports ROS2-style logging: DEBUG, INFO, WARN, ERROR, FATAL
 */

const LOG_LEVELS = {
  DEBUG: 0,
  INFO: 1,
  WARN: 2,
  ERROR: 3,
  FATAL: 4
};

export class LogManagerClass {
  constructor() {
    this.logs = [];
    this.maxLogs = 1000;
    this.listeners = [];
    this.defaultLevel = LOG_LEVELS.INFO;
    this.nodeLevels = new Map(); // Per-node log levels
    this.paused = false;
  }

  /**
   * Get a logger for a specific node
   * @param {string} nodeName - Node name
   * @returns {Object} Logger object with debug/info/warn/error/fatal methods
   */
  getLogger(nodeName) {
    return {
      debug: (msg) => this._log(nodeName, 'DEBUG', msg),
      info: (msg) => this._log(nodeName, 'INFO', msg),
      warn: (msg) => this._log(nodeName, 'WARN', msg),
      error: (msg) => this._log(nodeName, 'ERROR', msg),
      fatal: (msg) => this._log(nodeName, 'FATAL', msg)
    };
  }

  /**
   * Internal log method
   */
  _log(nodeName, level, message) {
    const levelValue = LOG_LEVELS[level];
    const nodeLevel = this.nodeLevels.get(nodeName) ?? this.defaultLevel;

    // Check if this log level should be output
    if (levelValue < nodeLevel) {
      return;
    }

    const entry = {
      timestamp: Date.now(),
      node: nodeName,
      level,
      levelValue,
      message
    };

    // Add to log buffer
    this.logs.push(entry);
    if (this.logs.length > this.maxLogs) {
      this.logs.shift();
    }

    // Console output with appropriate method
    const consoleMsg = `[${nodeName}] [${level}]: ${message}`;
    switch (level) {
      case 'DEBUG':
        console.debug(consoleMsg);
        break;
      case 'INFO':
        console.info(consoleMsg);
        break;
      case 'WARN':
        console.warn(consoleMsg);
        break;
      case 'ERROR':
      case 'FATAL':
        console.error(consoleMsg);
        break;
    }

    // Notify listeners
    if (!this.paused) {
      for (const listener of this.listeners) {
        try {
          listener(entry);
        } catch (e) {
          console.error('Error in log listener:', e);
        }
      }
    }
  }

  /**
   * Set log level for a specific node
   * @param {string} nodeName - Node name
   * @param {string} level - Log level (DEBUG, INFO, WARN, ERROR, FATAL)
   */
  setNodeLevel(nodeName, level) {
    const levelValue = LOG_LEVELS[level.toUpperCase()];
    if (levelValue !== undefined) {
      this.nodeLevels.set(nodeName, levelValue);
    }
  }

  /**
   * Set default log level
   * @param {string} level - Log level
   */
  setDefaultLevel(level) {
    const levelValue = LOG_LEVELS[level.toUpperCase()];
    if (levelValue !== undefined) {
      this.defaultLevel = levelValue;
    }
  }

  /**
   * Add a log listener
   * @param {Function} callback - Callback function(entry)
   * @returns {Function} Unsubscribe function
   */
  addListener(callback) {
    this.listeners.push(callback);
    return () => {
      const idx = this.listeners.indexOf(callback);
      if (idx >= 0) {
        this.listeners.splice(idx, 1);
      }
    };
  }

  /**
   * Get all logs
   * @param {Object} filter - Optional filter { level, node, since }
   * @returns {Array} Filtered logs
   */
  getLogs(filter = {}) {
    let result = [...this.logs];

    if (filter.level) {
      const minLevel = LOG_LEVELS[filter.level.toUpperCase()] ?? 0;
      result = result.filter(log => log.levelValue >= minLevel);
    }

    if (filter.node) {
      result = result.filter(log => log.node.includes(filter.node));
    }

    if (filter.since) {
      result = result.filter(log => log.timestamp >= filter.since);
    }

    return result;
  }

  /**
   * Clear all logs
   */
  clear() {
    this.logs = [];
  }

  /**
   * Pause log notifications to listeners
   */
  pause() {
    this.paused = true;
  }

  /**
   * Resume log notifications
   */
  resume() {
    this.paused = false;
  }

  /**
   * Format a log entry for display
   * @param {Object} entry - Log entry
   * @returns {string} Formatted string
   */
  formatEntry(entry) {
    const date = new Date(entry.timestamp);
    const timeStr = date.toLocaleTimeString('en-US', {
      hour12: false,
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    });
    const ms = String(entry.timestamp % 1000).padStart(3, '0');
    return `[${timeStr}.${ms}] [${entry.node}] [${entry.level}]: ${entry.message}`;
  }

  /**
   * Get available log levels
   */
  getLevels() {
    return Object.keys(LOG_LEVELS);
  }

  /**
   * Parse log level from string
   */
  parseLevel(levelStr) {
    return LOG_LEVELS[levelStr.toUpperCase()];
  }
}

// Singleton instance
export const LogManager = new LogManagerClass();

// Register with ServiceContainer for dependency injection
ServiceContainer.register('logManager', LogManager);
