/**
 * Message Registry - Central registry for all ROS2 message, service, and action types.
 * Message modules self-register by importing this singleton and calling register().
 */

class MessageRegistryClass {
  constructor() {
    this.messages = {};
    this.services = {};
    this.actions = {};
  }

  /**
   * Register a message type
   * @param {string} typeName - Full type name (e.g., 'geometry_msgs/msg/Twist')
   * @param {Object} definition - Message definition object
   */
  registerMessage(typeName, definition) {
    this.messages[typeName] = definition;
  }

  /**
   * Register a service type
   * @param {string} typeName - Full type name (e.g., 'turtlesim/srv/Spawn')
   * @param {Object} definition - Service definition object
   */
  registerService(typeName, definition) {
    this.services[typeName] = definition;
  }

  /**
   * Register an action type
   * @param {string} typeName - Full type name (e.g., 'turtlesim/action/RotateAbsolute')
   * @param {Object} definition - Action definition object
   */
  registerAction(typeName, definition) {
    this.actions[typeName] = definition;
  }

  /**
   * Get a message type definition
   */
  getMessage(typeName) {
    return this.messages[typeName] || null;
  }

  /**
   * Get a service type definition
   */
  getService(typeName) {
    return this.services[typeName] || null;
  }

  /**
   * Get an action type definition
   */
  getAction(typeName) {
    return this.actions[typeName] || null;
  }

  /**
   * Get any interface type (message, service, or action)
   */
  getInterface(typeName) {
    return this.messages[typeName] || this.services[typeName] || this.actions[typeName] || null;
  }

  /**
   * List all available message types
   */
  listMessages() {
    return Object.keys(this.messages);
  }

  /**
   * List all available service types
   */
  listServices() {
    return Object.keys(this.services);
  }

  /**
   * List all available action types
   */
  listActions() {
    return Object.keys(this.actions);
  }

  /**
   * Find message types matching a pattern
   */
  findMessages(pattern) {
    const regex = new RegExp(pattern, 'i');
    return Object.keys(this.messages).filter(name => regex.test(name));
  }

  /**
   * Find service types matching a pattern
   */
  findServices(pattern) {
    const regex = new RegExp(pattern, 'i');
    return Object.keys(this.services).filter(name => regex.test(name));
  }

  /**
   * Find action types matching a pattern
   */
  findActions(pattern) {
    const regex = new RegExp(pattern, 'i');
    return Object.keys(this.actions).filter(name => regex.test(name));
  }

  /**
   * Validate a message against its type definition
   */
  validateMessage(typeName, message) {
    const msgDef = this.messages[typeName];
    if (!msgDef) {
      return { valid: false, errors: [`Unknown message type: ${typeName}`] };
    }

    const errors = [];
    for (const [field, fieldDef] of Object.entries(msgDef.fields || {})) {
      if (message[field] === undefined) {
        if (fieldDef.default === undefined) {
          errors.push(`Missing required field: ${field}`);
        }
      }
    }

    return { valid: errors.length === 0, errors };
  }

  /**
   * Create a message with default values
   */
  createMessage(typeName, data = {}) {
    const msgDef = this.messages[typeName];
    if (!msgDef || !msgDef.create) {
      return null;
    }
    return msgDef.create(data);
  }

  /**
   * Get the definition string for a type (for ros2 interface show)
   */
  getDefinition(typeName) {
    const iface = this.getInterface(typeName);
    return iface?.definition || null;
  }
}

export const messageRegistry = new MessageRegistryClass();
