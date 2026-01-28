/**
 * Message Registry - Central registry for all ROS2 message, service, and action types
 *
 * This file imports all message modules (triggering self-registration) and re-exports
 * the registry methods for use by other parts of the codebase.
 */

// Import all message modules to trigger registration
import './loader.js';

// Import the registry
import { messageRegistry } from './registry.js';

// Re-export individual modules for backward compatibility
import std_msgs from './std_msgs.js';
import geometry_msgs from './geometry_msgs.js';
import turtlesim_msgs from './turtlesim_msgs.js';
import turtlesim_srvs from './turtlesim_srvs.js';
import turtlesim_actions from './turtlesim_actions.js';
import tf2_msgs from './tf2_msgs.js';
import sensor_msgs from './sensor_msgs.js';
import nav_msgs from './nav_msgs.js';

/**
 * Get a message type definition
 * @param {string} typeName - Full message type name (e.g., 'geometry_msgs/msg/Twist')
 * @returns {Object|null} Message definition or null if not found
 */
export function getMessage(typeName) {
  return messageRegistry.getMessage(typeName);
}

/**
 * Get a service type definition
 * @param {string} typeName - Full service type name (e.g., 'turtlesim/srv/Spawn')
 * @returns {Object|null} Service definition or null if not found
 */
export function getService(typeName) {
  return messageRegistry.getService(typeName);
}

/**
 * Get an action type definition
 * @param {string} typeName - Full action type name (e.g., 'turtlesim/action/RotateAbsolute')
 * @returns {Object|null} Action definition or null if not found
 */
export function getAction(typeName) {
  return messageRegistry.getAction(typeName);
}

/**
 * Get any interface type (message, service, or action)
 * @param {string} typeName - Full type name
 * @returns {Object|null} Interface definition or null if not found
 */
export function getInterface(typeName) {
  return messageRegistry.getInterface(typeName);
}

/**
 * List all available message types
 * @returns {string[]} Array of message type names
 */
export function listMessages() {
  return messageRegistry.listMessages();
}

/**
 * List all available service types
 * @returns {string[]} Array of service type names
 */
export function listServices() {
  return messageRegistry.listServices();
}

/**
 * List all available action types
 * @returns {string[]} Array of action type names
 */
export function listActions() {
  return messageRegistry.listActions();
}

/**
 * Find message types matching a pattern
 * @param {string} pattern - Search pattern
 * @returns {string[]} Matching type names
 */
export function findMessages(pattern) {
  return messageRegistry.findMessages(pattern);
}

/**
 * Find service types matching a pattern
 * @param {string} pattern - Search pattern
 * @returns {string[]} Matching type names
 */
export function findServices(pattern) {
  return messageRegistry.findServices(pattern);
}

/**
 * Find action types matching a pattern
 * @param {string} pattern - Search pattern
 * @returns {string[]} Matching type names
 */
export function findActions(pattern) {
  return messageRegistry.findActions(pattern);
}

/**
 * Validate a message against its type definition
 * @param {string} typeName - Message type name
 * @param {Object} message - Message to validate
 * @returns {{ valid: boolean, errors: string[] }}
 */
export function validateMessage(typeName, message) {
  return messageRegistry.validateMessage(typeName, message);
}

/**
 * Create a message with default values
 * @param {string} typeName - Message type name
 * @param {Object} data - Optional partial data
 * @returns {Object|null} Created message or null if type not found
 */
export function createMessage(typeName, data = {}) {
  return messageRegistry.createMessage(typeName, data);
}

/**
 * Get the definition string for a type (for ros2 interface show)
 * @param {string} typeName - Type name
 * @returns {string|null} Definition string or null if not found
 */
export function getDefinition(typeName) {
  return messageRegistry.getDefinition(typeName);
}

// Export modules for direct access
export { std_msgs, geometry_msgs, turtlesim_msgs, turtlesim_srvs, turtlesim_actions, tf2_msgs, sensor_msgs, nav_msgs };

// Export the registry for advanced use
export { messageRegistry };

// Default export
export default {
  getMessage,
  getService,
  getAction,
  getInterface,
  listMessages,
  listServices,
  listActions,
  findMessages,
  findServices,
  findActions,
  validateMessage,
  createMessage,
  getDefinition,
  messageRegistry
};
