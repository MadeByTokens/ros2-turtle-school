import { SimDDS } from './SimDDS.js';
import { LogManager } from './LogManager.js';
import { ServiceContainer } from './ServiceContainer.js';

/**
 * Base class for all simulated ROS2 nodes
 *
 * Supports dependency injection for testing:
 *   new Node('test', { simDDS: mockDDS, logManager: mockLogger })
 */
export class Node {
  constructor(name, options = {}) {
    this.name = name;
    this.namespace = options.namespace || '';
    this.fullName = this.namespace ? `${this.namespace}/${name}` : `/${name}`;
    this.publishers = [];
    this.subscriptions = [];
    this.services = [];
    this.actionServers = [];
    this.timers = [];
    this.parameters = new Map();
    this.running = false;

    // Support dependency injection with fallback chain:
    // 1. Explicit option passed to constructor
    // 2. ServiceContainer registration
    // 3. Default singleton import
    this.simDDS = options.simDDS || ServiceContainer.get('simDDS') || SimDDS;
    this.logManager = options.logManager || ServiceContainer.get('logManager') || LogManager;
    this.logger = this.logManager.getLogger(this.fullName);

    // Register with SimDDS
    this.simDDS.registerNode(this.fullName, {
      name: this.name,
      namespace: this.namespace
    });

    // Initialize default parameters
    if (options.parameters) {
      for (const [key, value] of Object.entries(options.parameters)) {
        this.declareParameter(key, value);
      }
    }
  }

  /**
   * Declare a parameter with default value
   */
  declareParameter(name, defaultValue) {
    if (!this.parameters.has(name)) {
      this.parameters.set(name, defaultValue);
    }
    return this.parameters.get(name);
  }

  /**
   * Get a parameter value
   */
  getParameter(name) {
    return this.parameters.get(name);
  }

  /**
   * Set a parameter value
   */
  setParameter(name, value) {
    if (this.parameters.has(name)) {
      this.parameters.set(name, value);
      this.onParameterChange(name, value);
      return true;
    }
    return false;
  }

  /**
   * Get all parameter names
   */
  getParameterNames() {
    return Array.from(this.parameters.keys());
  }

  /**
   * Called when a parameter changes - override in subclasses
   */
  onParameterChange(name, value) {
    // Override in subclasses
  }

  /**
   * Create a publisher
   */
  createPublisher(topic, msgType) {
    const pub = this.simDDS.createPublisher(this.fullName, topic, msgType);
    this.publishers.push(pub);
    return pub;
  }

  /**
   * Create a subscription
   */
  createSubscription(topic, msgType, callback) {
    const sub = this.simDDS.createSubscription(this.fullName, topic, msgType, callback.bind(this));
    this.subscriptions.push(sub);
    return sub;
  }

  /**
   * Create a service server
   */
  createService(serviceName, srvType, handler) {
    const srv = this.simDDS.createService(this.fullName, serviceName, srvType, handler.bind(this));
    this.services.push(srv);
    return srv;
  }

  /**
   * Create an action server
   */
  createActionServer(actionName, actionType, handlers) {
    const boundHandlers = {
      executeCallback: handlers.executeCallback.bind(this),
      goalCallback: handlers.goalCallback?.bind(this),
      cancelCallback: handlers.cancelCallback?.bind(this)
    };
    const actionServer = this.simDDS.createActionServer(this.fullName, actionName, actionType, boundHandlers);
    this.actionServers.push(actionServer);
    return actionServer;
  }

  /**
   * Create a timer
   */
  createTimer(periodMs, callback) {
    const timerId = setInterval(() => {
      if (this.running) {
        callback.call(this);
      }
    }, periodMs);
    this.timers.push(timerId);
    return timerId;
  }

  /**
   * Cancel a timer
   */
  cancelTimer(timerId) {
    clearInterval(timerId);
    const idx = this.timers.indexOf(timerId);
    if (idx >= 0) {
      this.timers.splice(idx, 1);
    }
  }

  /**
   * Get the current time
   */
  now() {
    return {
      sec: Math.floor(Date.now() / 1000),
      nanosec: (Date.now() % 1000) * 1000000
    };
  }

  /**
   * Log methods
   */
  logDebug(msg) {
    this.logger.debug(msg);
  }

  logInfo(msg) {
    this.logger.info(msg);
  }

  logWarn(msg) {
    this.logger.warn(msg);
  }

  logError(msg) {
    this.logger.error(msg);
  }

  logFatal(msg) {
    this.logger.fatal(msg);
  }

  /**
   * Called when node is started - override in subclasses
   */
  onInit() {
    // Override in subclasses
  }

  /**
   * Called when node is about to be destroyed - override in subclasses
   */
  onShutdown() {
    // Override in subclasses
  }

  /**
   * Start the node
   */
  start() {
    this.running = true;
    this.onInit();
    this.logInfo('Node started');
  }

  /**
   * Stop and destroy the node
   */
  destroy() {
    this.running = false;
    this.onShutdown();

    // Cancel all timers (timers are local, not managed by SimDDS)
    for (const timerId of this.timers) {
      clearInterval(timerId);
    }
    this.timers = [];

    // Log before unregister so the logger still works
    this.logInfo('Node destroyed');

    // Clear local arrays without calling individual destroy() methods.
    // SimDDS.unregisterNode() owns the comm cleanup to avoid double-cleanup.
    this.publishers = [];
    this.subscriptions = [];
    this.services = [];
    this.actionServers = [];

    // Unregister from DDS (handles all comm cleanup)
    this.simDDS.unregisterNode(this.fullName);
  }
}
