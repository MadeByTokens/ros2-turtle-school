/**
 * Abstract communication interface for ROS2 pub/sub/service.
 * Implement this interface to create different communication backends
 * (local, WebRTC, WebSocket, etc.)
 */
export class CommInterface {
  /**
   * Subscribe to a topic
   * @param {string} topic - Topic name
   * @param {string} msgType - Message type
   * @param {Function} callback - Callback function(msg)
   * @returns {string} Subscription ID
   */
  subscribe(topic, msgType, callback) {
    throw new Error('subscribe() must be implemented');
  }

  /**
   * Unsubscribe from a topic
   * @param {string} subscriptionId - Subscription ID returned from subscribe()
   */
  unsubscribe(subscriptionId) {
    throw new Error('unsubscribe() must be implemented');
  }

  /**
   * Publish a message to a topic
   * @param {string} topic - Topic name
   * @param {string} msgType - Message type
   * @param {Object} message - Message data
   */
  publish(topic, msgType, message) {
    throw new Error('publish() must be implemented');
  }

  /**
   * Advertise a topic for publishing
   * @param {string} topic - Topic name
   * @param {string} msgType - Message type
   * @param {string} nodeId - Publisher node ID
   * @returns {string} Publisher ID
   */
  advertise(topic, msgType, nodeId) {
    throw new Error('advertise() must be implemented');
  }

  /**
   * Unadvertise a topic
   * @param {string} publisherId - Publisher ID
   */
  unadvertise(publisherId) {
    throw new Error('unadvertise() must be implemented');
  }

  /**
   * Register a service server
   * @param {string} serviceName - Service name
   * @param {string} srvType - Service type
   * @param {Function} handler - Handler function(request) => response
   * @param {string} nodeId - Server node ID
   * @returns {string} Service ID
   */
  advertiseService(serviceName, srvType, handler, nodeId) {
    throw new Error('advertiseService() must be implemented');
  }

  /**
   * Unregister a service server
   * @param {string} serviceId - Service ID
   */
  unadvertiseService(serviceId) {
    throw new Error('unadvertiseService() must be implemented');
  }

  /**
   * Call a service
   * @param {string} serviceName - Service name
   * @param {string} srvType - Service type
   * @param {Object} request - Request data
   * @returns {Promise<Object>} Response
   */
  callService(serviceName, srvType, request) {
    throw new Error('callService() must be implemented');
  }

  /**
   * Register an action server
   * @param {string} actionName - Action name
   * @param {string} actionType - Action type
   * @param {Object} handlers - { executeCallback, goalCallback, cancelCallback }
   * @param {string} nodeId - Server node ID
   * @returns {string} Action server ID
   */
  advertiseAction(actionName, actionType, handlers, nodeId) {
    throw new Error('advertiseAction() must be implemented');
  }

  /**
   * Unregister an action server
   * @param {string} actionId - Action server ID
   */
  unadvertiseAction(actionId) {
    throw new Error('unadvertiseAction() must be implemented');
  }

  /**
   * Send an action goal
   * @param {string} actionName - Action name
   * @param {string} actionType - Action type
   * @param {Object} goal - Goal data
   * @param {Function} feedbackCallback - Optional feedback callback
   * @returns {Promise<Object>} Result
   */
  sendActionGoal(actionName, actionType, goal, feedbackCallback) {
    throw new Error('sendActionGoal() must be implemented');
  }

  /**
   * Get list of all topics
   * @returns {Array<{name: string, type: string, publishers: Array, subscribers: Array}>}
   */
  getTopics() {
    throw new Error('getTopics() must be implemented');
  }

  /**
   * Get list of all services
   * @returns {Array<{name: string, type: string, node: string}>}
   */
  getServices() {
    throw new Error('getServices() must be implemented');
  }

  /**
   * Get list of all actions
   * @returns {Array<{name: string, type: string, node: string}>}
   */
  getActions() {
    throw new Error('getActions() must be implemented');
  }

  /**
   * Cleanup and destroy the communication layer
   */
  destroy() {
    throw new Error('destroy() must be implemented');
  }
}
