/**
 * std_msgs - Standard ROS2 message types
 */

import { messageRegistry } from './registry.js';

export const Header = {
  name: 'std_msgs/msg/Header',
  fields: {
    stamp: { type: 'builtin_interfaces/msg/Time', default: { sec: 0, nanosec: 0 } },
    frame_id: { type: 'string', default: '' }
  },
  create: (data = {}) => ({
    stamp: data.stamp || { sec: 0, nanosec: 0 },
    frame_id: data.frame_id || ''
  }),
  definition: `# Standard metadata for higher-level stamped data types.
builtin_interfaces/Time stamp
string frame_id`
};

export const String = {
  name: 'std_msgs/msg/String',
  fields: {
    data: { type: 'string', default: '' }
  },
  create: (data = {}) => ({
    data: data.data || ''
  }),
  definition: `# A simple string message.
string data`
};

export const Bool = {
  name: 'std_msgs/msg/Bool',
  fields: {
    data: { type: 'bool', default: false }
  },
  create: (data = {}) => ({
    data: data.data || false
  }),
  definition: `# A simple boolean message.
bool data`
};

export const Int32 = {
  name: 'std_msgs/msg/Int32',
  fields: {
    data: { type: 'int32', default: 0 }
  },
  create: (data = {}) => ({
    data: data.data || 0
  }),
  definition: `# A 32-bit signed integer.
int32 data`
};

export const Int64 = {
  name: 'std_msgs/msg/Int64',
  fields: {
    data: { type: 'int64', default: 0 }
  },
  create: (data = {}) => ({
    data: data.data || 0
  }),
  definition: `# A 64-bit signed integer.
int64 data`
};

export const Float32 = {
  name: 'std_msgs/msg/Float32',
  fields: {
    data: { type: 'float32', default: 0.0 }
  },
  create: (data = {}) => ({
    data: data.data || 0.0
  }),
  definition: `# A 32-bit floating point number.
float32 data`
};

export const Float64 = {
  name: 'std_msgs/msg/Float64',
  fields: {
    data: { type: 'float64', default: 0.0 }
  },
  create: (data = {}) => ({
    data: data.data || 0.0
  }),
  definition: `# A 64-bit floating point number.
float64 data`
};

export const Empty = {
  name: 'std_msgs/msg/Empty',
  fields: {},
  create: () => ({}),
  definition: `# Empty message type.`
};

export const ColorRGBA = {
  name: 'std_msgs/msg/ColorRGBA',
  fields: {
    r: { type: 'float32', default: 0.0 },
    g: { type: 'float32', default: 0.0 },
    b: { type: 'float32', default: 0.0 },
    a: { type: 'float32', default: 1.0 }
  },
  create: (data = {}) => ({
    r: data.r ?? 0.0,
    g: data.g ?? 0.0,
    b: data.b ?? 0.0,
    a: data.a ?? 1.0
  }),
  definition: `# RGBA color with float values.
float32 r
float32 g
float32 b
float32 a`
};

// Self-register all message types
messageRegistry.registerMessage('std_msgs/msg/Header', Header);
messageRegistry.registerMessage('std_msgs/msg/String', String);
messageRegistry.registerMessage('std_msgs/msg/Bool', Bool);
messageRegistry.registerMessage('std_msgs/msg/Int32', Int32);
messageRegistry.registerMessage('std_msgs/msg/Int64', Int64);
messageRegistry.registerMessage('std_msgs/msg/Float32', Float32);
messageRegistry.registerMessage('std_msgs/msg/Float64', Float64);
messageRegistry.registerMessage('std_msgs/msg/Empty', Empty);
messageRegistry.registerMessage('std_msgs/msg/ColorRGBA', ColorRGBA);

// Export all message types
export default {
  Header,
  String,
  Bool,
  Int32,
  Int64,
  Float32,
  Float64,
  Empty,
  ColorRGBA
};
