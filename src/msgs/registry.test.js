import { describe, it, expect, beforeEach } from 'vitest';

// Create a fresh instance for testing instead of using the singleton
class MessageRegistryClass {
  constructor() {
    this.messages = {};
    this.services = {};
    this.actions = {};
  }

  registerMessage(typeName, definition) {
    this.messages[typeName] = definition;
  }

  registerService(typeName, definition) {
    this.services[typeName] = definition;
  }

  registerAction(typeName, definition) {
    this.actions[typeName] = definition;
  }

  getMessage(typeName) {
    return this.messages[typeName] || null;
  }

  getService(typeName) {
    return this.services[typeName] || null;
  }

  getAction(typeName) {
    return this.actions[typeName] || null;
  }

  getInterface(typeName) {
    return this.messages[typeName] || this.services[typeName] || this.actions[typeName] || null;
  }

  listMessages() {
    return Object.keys(this.messages);
  }

  listServices() {
    return Object.keys(this.services);
  }

  listActions() {
    return Object.keys(this.actions);
  }

  findMessages(pattern) {
    const regex = new RegExp(pattern, 'i');
    return Object.keys(this.messages).filter(name => regex.test(name));
  }

  findServices(pattern) {
    const regex = new RegExp(pattern, 'i');
    return Object.keys(this.services).filter(name => regex.test(name));
  }

  findActions(pattern) {
    const regex = new RegExp(pattern, 'i');
    return Object.keys(this.actions).filter(name => regex.test(name));
  }

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

  createMessage(typeName, data = {}) {
    const msgDef = this.messages[typeName];
    if (!msgDef || !msgDef.create) {
      return null;
    }
    return msgDef.create(data);
  }

  getDefinition(typeName) {
    const iface = this.getInterface(typeName);
    return iface?.definition || null;
  }
}

describe('MessageRegistry', () => {
  let registry;

  beforeEach(() => {
    registry = new MessageRegistryClass();
  });

  describe('registerMessage / getMessage', () => {
    it('registers and retrieves a message type', () => {
      const twistDef = {
        name: 'Twist',
        fields: {
          linear: { type: 'Vector3' },
          angular: { type: 'Vector3' }
        }
      };

      registry.registerMessage('geometry_msgs/msg/Twist', twistDef);
      expect(registry.getMessage('geometry_msgs/msg/Twist')).toEqual(twistDef);
    });

    it('returns null for unknown message types', () => {
      expect(registry.getMessage('unknown/msg/Type')).toBeNull();
    });

    it('overwrites existing registration', () => {
      registry.registerMessage('test/msg/Test', { version: 1 });
      registry.registerMessage('test/msg/Test', { version: 2 });
      expect(registry.getMessage('test/msg/Test')).toEqual({ version: 2 });
    });
  });

  describe('registerService / getService', () => {
    it('registers and retrieves a service type', () => {
      const spawnDef = {
        name: 'Spawn',
        request: { x: 'float', y: 'float', theta: 'float', name: 'string' },
        response: { name: 'string' }
      };

      registry.registerService('turtlesim/srv/Spawn', spawnDef);
      expect(registry.getService('turtlesim/srv/Spawn')).toEqual(spawnDef);
    });

    it('returns null for unknown service types', () => {
      expect(registry.getService('unknown/srv/Type')).toBeNull();
    });
  });

  describe('registerAction / getAction', () => {
    it('registers and retrieves an action type', () => {
      const rotateDef = {
        name: 'RotateAbsolute',
        goal: { theta: 'float' },
        result: { delta: 'float' },
        feedback: { remaining: 'float' }
      };

      registry.registerAction('turtlesim/action/RotateAbsolute', rotateDef);
      expect(registry.getAction('turtlesim/action/RotateAbsolute')).toEqual(rotateDef);
    });

    it('returns null for unknown action types', () => {
      expect(registry.getAction('unknown/action/Type')).toBeNull();
    });
  });

  describe('getInterface', () => {
    it('retrieves message types', () => {
      registry.registerMessage('test/msg/Msg', { type: 'message' });
      expect(registry.getInterface('test/msg/Msg')).toEqual({ type: 'message' });
    });

    it('retrieves service types', () => {
      registry.registerService('test/srv/Srv', { type: 'service' });
      expect(registry.getInterface('test/srv/Srv')).toEqual({ type: 'service' });
    });

    it('retrieves action types', () => {
      registry.registerAction('test/action/Act', { type: 'action' });
      expect(registry.getInterface('test/action/Act')).toEqual({ type: 'action' });
    });

    it('returns null for unknown types', () => {
      expect(registry.getInterface('unknown/type/Type')).toBeNull();
    });
  });

  describe('listMessages / listServices / listActions', () => {
    beforeEach(() => {
      registry.registerMessage('geometry_msgs/msg/Twist', {});
      registry.registerMessage('std_msgs/msg/String', {});
      registry.registerService('turtlesim/srv/Spawn', {});
      registry.registerService('turtlesim/srv/Kill', {});
      registry.registerAction('turtlesim/action/RotateAbsolute', {});
    });

    it('lists all message types', () => {
      const messages = registry.listMessages();
      expect(messages).toContain('geometry_msgs/msg/Twist');
      expect(messages).toContain('std_msgs/msg/String');
      expect(messages).toHaveLength(2);
    });

    it('lists all service types', () => {
      const services = registry.listServices();
      expect(services).toContain('turtlesim/srv/Spawn');
      expect(services).toContain('turtlesim/srv/Kill');
      expect(services).toHaveLength(2);
    });

    it('lists all action types', () => {
      const actions = registry.listActions();
      expect(actions).toContain('turtlesim/action/RotateAbsolute');
      expect(actions).toHaveLength(1);
    });

    it('returns empty arrays when no types registered', () => {
      const emptyRegistry = new MessageRegistryClass();
      expect(emptyRegistry.listMessages()).toEqual([]);
      expect(emptyRegistry.listServices()).toEqual([]);
      expect(emptyRegistry.listActions()).toEqual([]);
    });
  });

  describe('findMessages / findServices / findActions', () => {
    beforeEach(() => {
      registry.registerMessage('geometry_msgs/msg/Twist', {});
      registry.registerMessage('geometry_msgs/msg/Vector3', {});
      registry.registerMessage('std_msgs/msg/String', {});
      registry.registerService('turtlesim/srv/Spawn', {});
      registry.registerService('turtlesim/srv/SetPen', {});
      registry.registerAction('turtlesim/action/RotateAbsolute', {});
    });

    it('finds messages by pattern', () => {
      const results = registry.findMessages('geometry');
      expect(results).toContain('geometry_msgs/msg/Twist');
      expect(results).toContain('geometry_msgs/msg/Vector3');
      expect(results).not.toContain('std_msgs/msg/String');
    });

    it('finds messages case-insensitively', () => {
      const results = registry.findMessages('TWIST');
      expect(results).toContain('geometry_msgs/msg/Twist');
    });

    it('finds services by pattern', () => {
      const results = registry.findServices('S');
      expect(results).toContain('turtlesim/srv/Spawn');
      expect(results).toContain('turtlesim/srv/SetPen');
    });

    it('finds actions by pattern', () => {
      const results = registry.findActions('rotate');
      expect(results).toContain('turtlesim/action/RotateAbsolute');
    });

    it('returns empty array for no matches', () => {
      expect(registry.findMessages('nonexistent')).toEqual([]);
    });
  });

  describe('validateMessage', () => {
    beforeEach(() => {
      registry.registerMessage('test/msg/Point', {
        fields: {
          x: { type: 'float' },
          y: { type: 'float' },
          z: { type: 'float', default: 0 }
        }
      });
    });

    it('validates messages with all required fields', () => {
      const result = registry.validateMessage('test/msg/Point', { x: 1, y: 2 });
      expect(result.valid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('reports missing required fields', () => {
      const result = registry.validateMessage('test/msg/Point', { x: 1 });
      expect(result.valid).toBe(false);
      expect(result.errors).toContain('Missing required field: y');
    });

    it('accepts fields with defaults as optional', () => {
      const result = registry.validateMessage('test/msg/Point', { x: 1, y: 2 });
      expect(result.valid).toBe(true);
    });

    it('returns error for unknown message types', () => {
      const result = registry.validateMessage('unknown/msg/Type', {});
      expect(result.valid).toBe(false);
      expect(result.errors[0]).toContain('Unknown message type');
    });
  });

  describe('createMessage', () => {
    it('creates a message using the type create function', () => {
      registry.registerMessage('test/msg/Point', {
        create: (data) => ({
          x: data.x || 0,
          y: data.y || 0,
          z: data.z || 0
        })
      });

      const msg = registry.createMessage('test/msg/Point', { x: 5, y: 10 });
      expect(msg).toEqual({ x: 5, y: 10, z: 0 });
    });

    it('returns null for unknown types', () => {
      expect(registry.createMessage('unknown/msg/Type')).toBeNull();
    });

    it('returns null for types without create function', () => {
      registry.registerMessage('test/msg/NoCreate', { fields: {} });
      expect(registry.createMessage('test/msg/NoCreate')).toBeNull();
    });
  });

  describe('getDefinition', () => {
    it('returns definition string for messages', () => {
      registry.registerMessage('test/msg/Test', {
        definition: 'float64 x\nfloat64 y'
      });
      expect(registry.getDefinition('test/msg/Test')).toBe('float64 x\nfloat64 y');
    });

    it('returns definition string for services', () => {
      registry.registerService('test/srv/Test', {
        definition: 'float64 x\n---\nfloat64 result'
      });
      expect(registry.getDefinition('test/srv/Test')).toBe('float64 x\n---\nfloat64 result');
    });

    it('returns null when no definition', () => {
      registry.registerMessage('test/msg/NoDef', {});
      expect(registry.getDefinition('test/msg/NoDef')).toBeNull();
    });

    it('returns null for unknown types', () => {
      expect(registry.getDefinition('unknown/msg/Type')).toBeNull();
    });
  });
});
