import { SimDDS } from './SimDDS.js';

/**
 * ParamServer - Centralized parameter storage with per-node namespacing
 * Mirrors ROS2's parameter server behavior
 */
class ParamServerClass {
  constructor() {
    // Parameters stored as nodeName -> paramName -> value
    this.parameters = new Map();
  }

  /**
   * Get all parameters for a node
   * @param {string} nodeName - Full node name (e.g., '/turtlesim')
   * @returns {Map} Parameter map
   */
  getNodeParameters(nodeName) {
    const nodeInfo = SimDDS.getNodeInfo(nodeName);
    if (nodeInfo) {
      // Get parameters from the node instance
      const proc = this._getNodeProcess(nodeName);
      if (proc && proc.node.parameters) {
        return proc.node.parameters;
      }
    }
    return this.parameters.get(nodeName) || new Map();
  }

  /**
   * Get a specific parameter
   * @param {string} nodeName - Full node name
   * @param {string} paramName - Parameter name
   * @returns {*} Parameter value or undefined
   */
  getParameter(nodeName, paramName) {
    const proc = this._getNodeProcess(nodeName);
    if (proc && proc.node) {
      return proc.node.getParameter(paramName);
    }
    const nodeParams = this.parameters.get(nodeName);
    return nodeParams?.get(paramName);
  }

  /**
   * Set a parameter
   * @param {string} nodeName - Full node name
   * @param {string} paramName - Parameter name
   * @param {*} value - Parameter value
   * @returns {boolean} True if successful
   */
  setParameter(nodeName, paramName, value) {
    const proc = this._getNodeProcess(nodeName);
    if (proc && proc.node) {
      return proc.node.setParameter(paramName, value);
    }

    // Fallback to local storage
    if (!this.parameters.has(nodeName)) {
      this.parameters.set(nodeName, new Map());
    }
    this.parameters.get(nodeName).set(paramName, value);
    return true;
  }

  /**
   * List all parameter names for a node
   * @param {string} nodeName - Full node name
   * @returns {Array<string>} Parameter names
   */
  listParameters(nodeName) {
    const proc = this._getNodeProcess(nodeName);
    if (proc && proc.node) {
      return proc.node.getParameterNames();
    }
    const nodeParams = this.parameters.get(nodeName);
    return nodeParams ? Array.from(nodeParams.keys()) : [];
  }

  /**
   * Dump all parameters for a node as an object
   * @param {string} nodeName - Full node name
   * @returns {Object} Parameters as key-value object
   */
  dumpParameters(nodeName) {
    const result = {};
    const proc = this._getNodeProcess(nodeName);

    if (proc && proc.node) {
      for (const name of proc.node.getParameterNames()) {
        result[name] = proc.node.getParameter(name);
      }
    } else {
      const nodeParams = this.parameters.get(nodeName);
      if (nodeParams) {
        for (const [name, value] of nodeParams) {
          result[name] = value;
        }
      }
    }
    return result;
  }

  /**
   * Load parameters from an object
   * @param {string} nodeName - Full node name
   * @param {Object} params - Parameters to load
   * @returns {number} Number of parameters loaded
   */
  loadParameters(nodeName, params) {
    let count = 0;
    for (const [name, value] of Object.entries(params)) {
      if (this.setParameter(nodeName, name, value)) {
        count++;
      }
    }
    return count;
  }

  /**
   * List all nodes with parameters
   * @returns {Array<string>} Node names
   */
  listNodes() {
    const nodes = new Set();

    // Get nodes from SimDDS
    for (const nodeName of SimDDS.getNodes()) {
      const proc = this._getNodeProcess(nodeName);
      if (proc && proc.node && proc.node.getParameterNames().length > 0) {
        nodes.add(nodeName);
      }
    }

    // Add nodes from local storage
    for (const nodeName of this.parameters.keys()) {
      nodes.add(nodeName);
    }

    return Array.from(nodes);
  }

  /**
   * Helper to get process for a node
   */
  _getNodeProcess(nodeName) {
    // Import dynamically to avoid circular dependency
    const { ProcessManager } = require('./ProcessManager.js');
    return ProcessManager.getProcessByNodeName(nodeName);
  }
}

// Singleton instance
export const ParamServer = new ParamServerClass();

// Re-implement _getNodeProcess without require (ES modules)
import { ProcessManager } from './ProcessManager.js';
ParamServerClass.prototype._getNodeProcess = function(nodeName) {
  return ProcessManager.getProcessByNodeName(nodeName);
};
