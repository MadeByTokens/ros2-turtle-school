import { LocalComm } from '../comm/LocalComm.js';

/**
 * SimDDS - Simulated DDS middleware singleton
 * Provides ROS2-like pub/sub, services, and actions
 */
class SimDDSClass {
  constructor() {
    this.comm = new LocalComm();
    this.nodes = new Map(); // nodeId -> node info
  }

  /**
   * Set a different communication provider
   * @param {CommInterface} commProvider - New communication provider
   */
  setCommProvider(commProvider) {
    // Cleanup old provider
    if (this.comm) {
      this.comm.destroy();
    }
    this.comm = commProvider;
  }

  /**
   * Register a node with the DDS
   * @param {string} nodeId - Node name/ID
   * @param {Object} nodeInfo - Additional node information
   */
  registerNode(nodeId, nodeInfo = {}) {
    this.nodes.set(nodeId, {
      ...nodeInfo,
      publishers: [],
      subscribers: [],
      services: [],
      actions: [],
      parameters: new Map()
    });
  }

  /**
   * Unregister a node from the DDS
   * @param {string} nodeId - Node name/ID
   */
  unregisterNode(nodeId) {
    const nodeData = this.nodes.get(nodeId);
    if (nodeData) {
      // Cleanup all publishers
      for (const pubId of nodeData.publishers) {
        this.comm.unadvertise(pubId);
      }
      // Cleanup all subscribers
      for (const subId of nodeData.subscribers) {
        this.comm.unsubscribe(subId);
      }
      // Cleanup all services
      for (const srvId of nodeData.services) {
        this.comm.unadvertiseService(srvId);
      }
      // Cleanup all actions
      for (const actionId of nodeData.actions) {
        this.comm.unadvertiseAction(actionId);
      }
      this.nodes.delete(nodeId);
    }
  }

  /**
   * Create a publisher
   * @param {string} nodeId - Node name
   * @param {string} topic - Topic name
   * @param {string} msgType - Message type
   * @returns {Object} Publisher object with publish() method
   */
  createPublisher(nodeId, topic, msgType) {
    const pubId = this.comm.advertise(topic, msgType, nodeId);
    const nodeData = this.nodes.get(nodeId);
    if (nodeData) {
      nodeData.publishers.push(pubId);
    }

    return {
      id: pubId,
      topic,
      msgType,
      publish: (message) => {
        this.comm.publish(topic, msgType, message);
      },
      destroy: () => {
        this.comm.unadvertise(pubId);
        if (nodeData) {
          const idx = nodeData.publishers.indexOf(pubId);
          if (idx >= 0) nodeData.publishers.splice(idx, 1);
        }
      }
    };
  }

  /**
   * Create a subscription
   * @param {string} nodeId - Node name
   * @param {string} topic - Topic name
   * @param {string} msgType - Message type
   * @param {Function} callback - Message callback
   * @returns {Object} Subscription object
   */
  createSubscription(nodeId, topic, msgType, callback) {
    const subId = this.comm.subscribe(topic, msgType, callback);
    const nodeData = this.nodes.get(nodeId);
    if (nodeData) {
      nodeData.subscribers.push(subId);
    }

    return {
      id: subId,
      topic,
      msgType,
      destroy: () => {
        this.comm.unsubscribe(subId);
        if (nodeData) {
          const idx = nodeData.subscribers.indexOf(subId);
          if (idx >= 0) nodeData.subscribers.splice(idx, 1);
        }
      }
    };
  }

  /**
   * Create a service server
   * @param {string} nodeId - Node name
   * @param {string} serviceName - Service name
   * @param {string} srvType - Service type
   * @param {Function} handler - Request handler
   * @returns {Object} Service object
   */
  createService(nodeId, serviceName, srvType, handler) {
    const srvId = this.comm.advertiseService(serviceName, srvType, handler, nodeId);
    const nodeData = this.nodes.get(nodeId);
    if (nodeData) {
      nodeData.services.push(srvId);
    }

    return {
      id: srvId,
      name: serviceName,
      type: srvType,
      destroy: () => {
        this.comm.unadvertiseService(srvId);
        if (nodeData) {
          const idx = nodeData.services.indexOf(srvId);
          if (idx >= 0) nodeData.services.splice(idx, 1);
        }
      }
    };
  }

  /**
   * Call a service
   * @param {string} serviceName - Service name
   * @param {string} srvType - Service type
   * @param {Object} request - Request data
   * @returns {Promise<Object>} Response
   */
  async callService(serviceName, srvType, request) {
    return this.comm.callService(serviceName, srvType, request);
  }

  /**
   * Create an action server
   * @param {string} nodeId - Node name
   * @param {string} actionName - Action name
   * @param {string} actionType - Action type
   * @param {Object} handlers - Action handlers
   * @returns {Object} Action server object
   */
  createActionServer(nodeId, actionName, actionType, handlers) {
    const actionId = this.comm.advertiseAction(actionName, actionType, handlers, nodeId);
    const nodeData = this.nodes.get(nodeId);
    if (nodeData) {
      nodeData.actions.push(actionId);
    }

    return {
      id: actionId,
      name: actionName,
      type: actionType,
      destroy: () => {
        this.comm.unadvertiseAction(actionId);
        if (nodeData) {
          const idx = nodeData.actions.indexOf(actionId);
          if (idx >= 0) nodeData.actions.splice(idx, 1);
        }
      }
    };
  }

  /**
   * Send an action goal
   * @param {string} actionName - Action name
   * @param {string} actionType - Action type
   * @param {Object} goal - Goal data
   * @param {Function} feedbackCallback - Feedback callback
   * @returns {Promise<Object>} Result
   */
  async sendActionGoal(actionName, actionType, goal, feedbackCallback) {
    return this.comm.sendActionGoal(actionName, actionType, goal, feedbackCallback);
  }

  /**
   * Publish a message (convenience method)
   */
  publish(topic, msgType, message) {
    this.comm.publish(topic, msgType, message);
  }

  /**
   * Subscribe to a topic (convenience method for CLI)
   */
  subscribe(topic, msgType, callback) {
    return this.comm.subscribe(topic, msgType, callback);
  }

  /**
   * Unsubscribe (convenience method)
   */
  unsubscribe(subId) {
    return this.comm.unsubscribe(subId);
  }

  /**
   * Get all registered nodes
   */
  getNodes() {
    return Array.from(this.nodes.keys());
  }

  /**
   * Get node information
   */
  getNodeInfo(nodeId) {
    return this.nodes.get(nodeId);
  }

  /**
   * Get all topics
   */
  getTopics() {
    return this.comm.getTopics();
  }

  /**
   * Get topic info
   */
  getTopicInfo(topicName) {
    const topics = this.comm.getTopics();
    return topics.find(t => t.name === topicName);
  }

  /**
   * Get all services
   */
  getServices() {
    return this.comm.getServices();
  }

  /**
   * Get service info
   */
  getServiceInfo(serviceName) {
    const services = this.comm.getServices();
    return services.find(s => s.name === serviceName);
  }

  /**
   * Get all actions
   */
  getActions() {
    return this.comm.getActions();
  }

  /**
   * Get action info
   */
  getActionInfo(actionName) {
    const actions = this.comm.getActions();
    return actions.find(a => a.name === actionName);
  }

  /**
   * Reset the DDS (clear all nodes, topics, services)
   */
  reset() {
    for (const nodeId of this.nodes.keys()) {
      this.unregisterNode(nodeId);
    }
    this.comm.destroy();
    this.comm = new LocalComm();
  }
}

// Singleton instance
export const SimDDS = new SimDDSClass();
