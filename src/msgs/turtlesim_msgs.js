/**
 * turtlesim_msgs - Turtlesim-specific message types
 */

export const Pose = {
  name: 'turtlesim/msg/Pose',
  fields: {
    x: { type: 'float32', default: 0.0 },
    y: { type: 'float32', default: 0.0 },
    theta: { type: 'float32', default: 0.0 },
    linear_velocity: { type: 'float32', default: 0.0 },
    angular_velocity: { type: 'float32', default: 0.0 }
  },
  create: (data = {}) => ({
    x: data.x ?? 0.0,
    y: data.y ?? 0.0,
    theta: data.theta ?? 0.0,
    linear_velocity: data.linear_velocity ?? 0.0,
    angular_velocity: data.angular_velocity ?? 0.0
  }),
  definition: `# Pose of a turtle in turtlesim.
float32 x
float32 y
float32 theta

float32 linear_velocity
float32 angular_velocity`
};

export const Color = {
  name: 'turtlesim/msg/Color',
  fields: {
    r: { type: 'uint8', default: 0 },
    g: { type: 'uint8', default: 0 },
    b: { type: 'uint8', default: 0 }
  },
  create: (data = {}) => ({
    r: data.r ?? 0,
    g: data.g ?? 0,
    b: data.b ?? 0
  }),
  definition: `# RGB color values.
uint8 r
uint8 g
uint8 b`
};

// Export all message types
export default {
  Pose,
  Color
};
