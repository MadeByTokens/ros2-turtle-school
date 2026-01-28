/**
 * Message Registry - Central registry for all ROS2 message, service, and action types
 */

import std_msgs from './std_msgs.js';
import geometry_msgs from './geometry_msgs.js';
import turtlesim_msgs from './turtlesim_msgs.js';
import turtlesim_srvs from './turtlesim_srvs.js';
import turtlesim_actions from './turtlesim_actions.js';
import tf2_msgs from './tf2_msgs.js';
import sensor_msgs from './sensor_msgs.js';
import nav_msgs from './nav_msgs.js';

// All message types registry
const messages = {
  // std_msgs
  'std_msgs/msg/Header': std_msgs.Header,
  'std_msgs/msg/String': std_msgs.String,
  'std_msgs/msg/Bool': std_msgs.Bool,
  'std_msgs/msg/Int32': std_msgs.Int32,
  'std_msgs/msg/Int64': std_msgs.Int64,
  'std_msgs/msg/Float32': std_msgs.Float32,
  'std_msgs/msg/Float64': std_msgs.Float64,
  'std_msgs/msg/Empty': std_msgs.Empty,
  'std_msgs/msg/ColorRGBA': std_msgs.ColorRGBA,

  // geometry_msgs
  'geometry_msgs/msg/Vector3': geometry_msgs.Vector3,
  'geometry_msgs/msg/Point': geometry_msgs.Point,
  'geometry_msgs/msg/Quaternion': geometry_msgs.Quaternion,
  'geometry_msgs/msg/Pose': geometry_msgs.Pose,
  'geometry_msgs/msg/PoseStamped': geometry_msgs.PoseStamped,
  'geometry_msgs/msg/Twist': geometry_msgs.Twist,
  'geometry_msgs/msg/TwistStamped': geometry_msgs.TwistStamped,
  'geometry_msgs/msg/Transform': geometry_msgs.Transform,
  'geometry_msgs/msg/TransformStamped': geometry_msgs.TransformStamped,

  // turtlesim
  'turtlesim/msg/Pose': turtlesim_msgs.Pose,
  'turtlesim/msg/Color': turtlesim_msgs.Color,

  // tf2_msgs
  'tf2_msgs/msg/TFMessage': tf2_msgs.TFMessage,

  // sensor_msgs
  'sensor_msgs/msg/LaserScan': sensor_msgs.LaserScan,
  'sensor_msgs/msg/Imu': sensor_msgs.Imu,

  // nav_msgs
  'nav_msgs/msg/MapMetaData': nav_msgs.MapMetaData,
  'nav_msgs/msg/OccupancyGrid': nav_msgs.OccupancyGrid,
  'nav_msgs/msg/Odometry': nav_msgs.Odometry,
  'nav_msgs/msg/Path': nav_msgs.Path
};

// All service types registry
const services = {
  'turtlesim/srv/Spawn': turtlesim_srvs.Spawn,
  'turtlesim/srv/Kill': turtlesim_srvs.Kill,
  'turtlesim/srv/SetPen': turtlesim_srvs.SetPen,
  'turtlesim/srv/TeleportAbsolute': turtlesim_srvs.TeleportAbsolute,
  'turtlesim/srv/TeleportRelative': turtlesim_srvs.TeleportRelative
};

// All action types registry
const actions = {
  'turtlesim/action/RotateAbsolute': turtlesim_actions.RotateAbsolute
};

/**
 * Get a message type definition
 * @param {string} typeName - Full message type name (e.g., 'geometry_msgs/msg/Twist')
 * @returns {Object|null} Message definition or null if not found
 */
export function getMessage(typeName) {
  return messages[typeName] || null;
}

/**
 * Get a service type definition
 * @param {string} typeName - Full service type name (e.g., 'turtlesim/srv/Spawn')
 * @returns {Object|null} Service definition or null if not found
 */
export function getService(typeName) {
  return services[typeName] || null;
}

/**
 * Get an action type definition
 * @param {string} typeName - Full action type name (e.g., 'turtlesim/action/RotateAbsolute')
 * @returns {Object|null} Action definition or null if not found
 */
export function getAction(typeName) {
  return actions[typeName] || null;
}

/**
 * Get any interface type (message, service, or action)
 * @param {string} typeName - Full type name
 * @returns {Object|null} Interface definition or null if not found
 */
export function getInterface(typeName) {
  return messages[typeName] || services[typeName] || actions[typeName] || null;
}

/**
 * List all available message types
 * @returns {string[]} Array of message type names
 */
export function listMessages() {
  return Object.keys(messages);
}

/**
 * List all available service types
 * @returns {string[]} Array of service type names
 */
export function listServices() {
  return Object.keys(services);
}

/**
 * List all available action types
 * @returns {string[]} Array of action type names
 */
export function listActions() {
  return Object.keys(actions);
}

/**
 * Find message types matching a pattern
 * @param {string} pattern - Search pattern
 * @returns {string[]} Matching type names
 */
export function findMessages(pattern) {
  const regex = new RegExp(pattern, 'i');
  return Object.keys(messages).filter(name => regex.test(name));
}

/**
 * Find service types matching a pattern
 * @param {string} pattern - Search pattern
 * @returns {string[]} Matching type names
 */
export function findServices(pattern) {
  const regex = new RegExp(pattern, 'i');
  return Object.keys(services).filter(name => regex.test(name));
}

/**
 * Find action types matching a pattern
 * @param {string} pattern - Search pattern
 * @returns {string[]} Matching type names
 */
export function findActions(pattern) {
  const regex = new RegExp(pattern, 'i');
  return Object.keys(actions).filter(name => regex.test(name));
}

/**
 * Validate a message against its type definition
 * @param {string} typeName - Message type name
 * @param {Object} message - Message to validate
 * @returns {{ valid: boolean, errors: string[] }}
 */
export function validateMessage(typeName, message) {
  const msgDef = messages[typeName];
  if (!msgDef) {
    return { valid: false, errors: [`Unknown message type: ${typeName}`] };
  }

  const errors = [];

  for (const [field, fieldDef] of Object.entries(msgDef.fields || {})) {
    if (message[field] === undefined) {
      // Field missing but has default - OK
      if (fieldDef.default === undefined) {
        errors.push(`Missing required field: ${field}`);
      }
    }
  }

  return {
    valid: errors.length === 0,
    errors
  };
}

/**
 * Create a message with default values
 * @param {string} typeName - Message type name
 * @param {Object} data - Optional partial data
 * @returns {Object|null} Created message or null if type not found
 */
export function createMessage(typeName, data = {}) {
  const msgDef = messages[typeName];
  if (!msgDef || !msgDef.create) {
    return null;
  }
  return msgDef.create(data);
}

/**
 * Get the definition string for a type (for ros2 interface show)
 * @param {string} typeName - Type name
 * @returns {string|null} Definition string or null if not found
 */
export function getDefinition(typeName) {
  const iface = getInterface(typeName);
  return iface?.definition || null;
}

// Export modules for direct access
export { std_msgs, geometry_msgs, turtlesim_msgs, turtlesim_srvs, turtlesim_actions, tf2_msgs, sensor_msgs, nav_msgs };

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
  messages,
  services,
  actions
};
