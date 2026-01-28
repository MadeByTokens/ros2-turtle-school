/**
 * geometry_msgs - Geometry-related ROS2 message types
 */

export const Vector3 = {
  name: 'geometry_msgs/msg/Vector3',
  fields: {
    x: { type: 'float64', default: 0.0 },
    y: { type: 'float64', default: 0.0 },
    z: { type: 'float64', default: 0.0 }
  },
  create: (data = {}) => ({
    x: data.x ?? 0.0,
    y: data.y ?? 0.0,
    z: data.z ?? 0.0
  }),
  definition: `# A 3D vector.
float64 x
float64 y
float64 z`
};

export const Point = {
  name: 'geometry_msgs/msg/Point',
  fields: {
    x: { type: 'float64', default: 0.0 },
    y: { type: 'float64', default: 0.0 },
    z: { type: 'float64', default: 0.0 }
  },
  create: (data = {}) => ({
    x: data.x ?? 0.0,
    y: data.y ?? 0.0,
    z: data.z ?? 0.0
  }),
  definition: `# A point in 3D space.
float64 x
float64 y
float64 z`
};

export const Quaternion = {
  name: 'geometry_msgs/msg/Quaternion',
  fields: {
    x: { type: 'float64', default: 0.0 },
    y: { type: 'float64', default: 0.0 },
    z: { type: 'float64', default: 0.0 },
    w: { type: 'float64', default: 1.0 }
  },
  create: (data = {}) => ({
    x: data.x ?? 0.0,
    y: data.y ?? 0.0,
    z: data.z ?? 0.0,
    w: data.w ?? 1.0
  }),
  definition: `# A quaternion representing orientation.
float64 x
float64 y
float64 z
float64 w`
};

export const Pose = {
  name: 'geometry_msgs/msg/Pose',
  fields: {
    position: { type: 'geometry_msgs/msg/Point' },
    orientation: { type: 'geometry_msgs/msg/Quaternion' }
  },
  create: (data = {}) => ({
    position: Point.create(data.position),
    orientation: Quaternion.create(data.orientation)
  }),
  definition: `# A representation of pose in free space.
Point position
Quaternion orientation`
};

export const PoseStamped = {
  name: 'geometry_msgs/msg/PoseStamped',
  fields: {
    header: { type: 'std_msgs/msg/Header' },
    pose: { type: 'geometry_msgs/msg/Pose' }
  },
  create: (data = {}) => ({
    header: data.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
    pose: Pose.create(data.pose)
  }),
  definition: `# A Pose with reference coordinate frame and timestamp.
std_msgs/Header header
Pose pose`
};

export const Twist = {
  name: 'geometry_msgs/msg/Twist',
  fields: {
    linear: { type: 'geometry_msgs/msg/Vector3' },
    angular: { type: 'geometry_msgs/msg/Vector3' }
  },
  create: (data = {}) => ({
    linear: Vector3.create(data.linear),
    angular: Vector3.create(data.angular)
  }),
  definition: `# Velocity in free space, broken into linear and angular parts.
Vector3 linear
Vector3 angular`
};

export const TwistStamped = {
  name: 'geometry_msgs/msg/TwistStamped',
  fields: {
    header: { type: 'std_msgs/msg/Header' },
    twist: { type: 'geometry_msgs/msg/Twist' }
  },
  create: (data = {}) => ({
    header: data.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
    twist: Twist.create(data.twist)
  }),
  definition: `# A twist with reference coordinate frame and timestamp.
std_msgs/Header header
Twist twist`
};

export const Transform = {
  name: 'geometry_msgs/msg/Transform',
  fields: {
    translation: { type: 'geometry_msgs/msg/Vector3' },
    rotation: { type: 'geometry_msgs/msg/Quaternion' }
  },
  create: (data = {}) => ({
    translation: Vector3.create(data.translation),
    rotation: Quaternion.create(data.rotation)
  }),
  definition: `# A transform between two coordinate frames.
Vector3 translation
Quaternion rotation`
};

export const TransformStamped = {
  name: 'geometry_msgs/msg/TransformStamped',
  fields: {
    header: { type: 'std_msgs/msg/Header' },
    child_frame_id: { type: 'string', default: '' },
    transform: { type: 'geometry_msgs/msg/Transform' }
  },
  create: (data = {}) => ({
    header: data.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
    child_frame_id: data.child_frame_id || '',
    transform: Transform.create(data.transform)
  }),
  definition: `# A transform with reference and child coordinate frames.
std_msgs/Header header
string child_frame_id
Transform transform`
};

// Export all message types
export default {
  Vector3,
  Point,
  Quaternion,
  Pose,
  PoseStamped,
  Twist,
  TwistStamped,
  Transform,
  TransformStamped
};
