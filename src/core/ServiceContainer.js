/**
 * ServiceContainer - Simple dependency injection container
 * Enables testability by allowing mock injection of core services.
 *
 * Usage:
 *   // Registration (done by service modules)
 *   ServiceContainer.register('logManager', LogManager);
 *
 *   // Retrieval (with fallback)
 *   const logger = options.logManager || ServiceContainer.get('logManager') || LogManager;
 *
 *   // Testing
 *   ServiceContainer.register('simDDS', mockSimDDS);
 *   // ... run tests ...
 *   ServiceContainer.reset();
 */
class ServiceContainerClass {
  constructor() {
    this._services = new Map();
  }

  /**
   * Register a service instance
   * @param {string} name - Service name
   * @param {*} instance - Service instance
   */
  register(name, instance) {
    this._services.set(name, instance);
  }

  /**
   * Get a service by name
   * @param {string} name - Service name
   * @returns {*} Service instance or undefined
   */
  get(name) {
    return this._services.get(name);
  }

  /**
   * Check if a service is registered
   * @param {string} name - Service name
   * @returns {boolean}
   */
  has(name) {
    return this._services.has(name);
  }

  /**
   * Reset container (useful for testing)
   */
  reset() {
    this._services.clear();
  }

  /**
   * Get all registered service names
   * @returns {string[]}
   */
  getServiceNames() {
    return Array.from(this._services.keys());
  }
}

export const ServiceContainer = new ServiceContainerClass();
