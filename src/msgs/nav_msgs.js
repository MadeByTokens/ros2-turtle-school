/**
 * nav_msgs - Navigation-related ROS2 message types
 */

export const MapMetaData = {
  name: 'nav_msgs/msg/MapMetaData',
  fields: {
    map_load_time: { type: 'builtin_interfaces/msg/Time', default: { sec: 0, nanosec: 0 } },
    resolution: { type: 'float32', default: 0.05 },
    width: { type: 'uint32', default: 0 },
    height: { type: 'uint32', default: 0 },
    origin: { type: 'geometry_msgs/msg/Pose' }
  },
  create: (data = {}) => ({
    map_load_time: data.map_load_time || { sec: 0, nanosec: 0 },
    resolution: data.resolution ?? 0.05,
    width: data.width ?? 0,
    height: data.height ?? 0,
    origin: data.origin || {
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    }
  }),
  definition: `# Metadata about a map.
builtin_interfaces/Time map_load_time
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin`
};

export const OccupancyGrid = {
  name: 'nav_msgs/msg/OccupancyGrid',
  fields: {
    header: { type: 'std_msgs/msg/Header' },
    info: { type: 'nav_msgs/msg/MapMetaData' },
    data: { type: 'int8[]', default: [] }
  },
  create: (data = {}) => ({
    header: data.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
    info: MapMetaData.create(data.info),
    data: data.data || []
  }),
  definition: `# 2D occupancy grid map.
# Values: -1 = unknown, 0 = free, 100 = occupied
std_msgs/Header header
MapMetaData info
int8[] data`
};

export const Odometry = {
  name: 'nav_msgs/msg/Odometry',
  fields: {
    header: { type: 'std_msgs/msg/Header' },
    child_frame_id: { type: 'string', default: '' },
    pose: { type: 'geometry_msgs/msg/PoseWithCovariance' },
    twist: { type: 'geometry_msgs/msg/TwistWithCovariance' }
  },
  create: (data = {}) => ({
    header: data.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: 'odom' },
    child_frame_id: data.child_frame_id || 'base_link',
    pose: data.pose || {
      pose: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 }
      },
      covariance: new Array(36).fill(0)
    },
    twist: data.twist || {
      twist: {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      },
      covariance: new Array(36).fill(0)
    }
  }),
  definition: `# Robot odometry.
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist`
};

export const Path = {
  name: 'nav_msgs/msg/Path',
  fields: {
    header: { type: 'std_msgs/msg/Header' },
    poses: { type: 'geometry_msgs/msg/PoseStamped[]', default: [] }
  },
  create: (data = {}) => ({
    header: data.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
    poses: data.poses || []
  }),
  definition: `# A path represented as a series of poses.
std_msgs/Header header
geometry_msgs/PoseStamped[] poses`
};

// Export all message types
export default {
  MapMetaData,
  OccupancyGrid,
  Odometry,
  Path
};
