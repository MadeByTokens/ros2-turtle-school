/**
 * turtlesim_srvs - Turtlesim service definitions
 */

export const Spawn = {
  name: 'turtlesim/srv/Spawn',
  type: 'service',
  request: {
    x: { type: 'float32', default: 0.0 },
    y: { type: 'float32', default: 0.0 },
    theta: { type: 'float32', default: 0.0 },
    name: { type: 'string', default: '' }
  },
  response: {
    name: { type: 'string' }
  },
  createRequest: (data = {}) => ({
    x: data.x ?? 5.544445,
    y: data.y ?? 5.544445,
    theta: data.theta ?? 0.0,
    name: data.name || ''
  }),
  createResponse: (data = {}) => ({
    name: data.name || ''
  }),
  definition: `# Spawn a new turtle.
float32 x
float32 y
float32 theta
string name
---
string name`
};

export const Kill = {
  name: 'turtlesim/srv/Kill',
  type: 'service',
  request: {
    name: { type: 'string', default: '' }
  },
  response: {},
  createRequest: (data = {}) => ({
    name: data.name || ''
  }),
  createResponse: () => ({}),
  definition: `# Kill a turtle by name.
string name
---`
};

export const SetPen = {
  name: 'turtlesim/srv/SetPen',
  type: 'service',
  request: {
    r: { type: 'uint8', default: 0 },
    g: { type: 'uint8', default: 0 },
    b: { type: 'uint8', default: 0 },
    width: { type: 'uint8', default: 1 },
    off: { type: 'uint8', default: 0 }
  },
  response: {},
  createRequest: (data = {}) => ({
    r: data.r ?? 255,
    g: data.g ?? 255,
    b: data.b ?? 255,
    width: data.width ?? 3,
    off: data.off ?? 0
  }),
  createResponse: () => ({}),
  definition: `# Set the pen parameters.
uint8 r
uint8 g
uint8 b
uint8 width
uint8 off
---`
};

export const TeleportAbsolute = {
  name: 'turtlesim/srv/TeleportAbsolute',
  type: 'service',
  request: {
    x: { type: 'float32', default: 0.0 },
    y: { type: 'float32', default: 0.0 },
    theta: { type: 'float32', default: 0.0 }
  },
  response: {},
  createRequest: (data = {}) => ({
    x: data.x ?? 0.0,
    y: data.y ?? 0.0,
    theta: data.theta ?? 0.0
  }),
  createResponse: () => ({}),
  definition: `# Teleport the turtle to an absolute position.
float32 x
float32 y
float32 theta
---`
};

export const TeleportRelative = {
  name: 'turtlesim/srv/TeleportRelative',
  type: 'service',
  request: {
    linear: { type: 'float32', default: 0.0 },
    angular: { type: 'float32', default: 0.0 }
  },
  response: {},
  createRequest: (data = {}) => ({
    linear: data.linear ?? 0.0,
    angular: data.angular ?? 0.0
  }),
  createResponse: () => ({}),
  definition: `# Teleport the turtle relative to its current position.
float32 linear
float32 angular
---`
};

// Export all service types
export default {
  Spawn,
  Kill,
  SetPen,
  TeleportAbsolute,
  TeleportRelative
};
