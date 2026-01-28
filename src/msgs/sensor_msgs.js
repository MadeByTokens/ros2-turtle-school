/**
 * sensor_msgs - Sensor-related ROS2 message types
 */

import { messageRegistry } from './registry.js';

export const LaserScan = {
  name: 'sensor_msgs/msg/LaserScan',
  fields: {
    header: { type: 'std_msgs/msg/Header' },
    angle_min: { type: 'float32', default: 0.0 },
    angle_max: { type: 'float32', default: 0.0 },
    angle_increment: { type: 'float32', default: 0.0 },
    time_increment: { type: 'float32', default: 0.0 },
    scan_time: { type: 'float32', default: 0.0 },
    range_min: { type: 'float32', default: 0.0 },
    range_max: { type: 'float32', default: 0.0 },
    ranges: { type: 'float32[]', default: [] },
    intensities: { type: 'float32[]', default: [] }
  },
  create: (data = {}) => ({
    header: data.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: 'laser' },
    angle_min: data.angle_min ?? -Math.PI,
    angle_max: data.angle_max ?? Math.PI,
    angle_increment: data.angle_increment ?? (Math.PI / 180),
    time_increment: data.time_increment ?? 0.0,
    scan_time: data.scan_time ?? 0.1,
    range_min: data.range_min ?? 0.1,
    range_max: data.range_max ?? 10.0,
    ranges: data.ranges || [],
    intensities: data.intensities || []
  }),
  definition: `# Single scan from a planar laser range-finder.
std_msgs/Header header

float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds]
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m]
float32[] intensities    # intensity data [device-specific units]`
};

export const Imu = {
  name: 'sensor_msgs/msg/Imu',
  fields: {
    header: { type: 'std_msgs/msg/Header' },
    orientation: { type: 'geometry_msgs/msg/Quaternion' },
    orientation_covariance: { type: 'float64[9]', default: new Array(9).fill(0) },
    angular_velocity: { type: 'geometry_msgs/msg/Vector3' },
    angular_velocity_covariance: { type: 'float64[9]', default: new Array(9).fill(0) },
    linear_acceleration: { type: 'geometry_msgs/msg/Vector3' },
    linear_acceleration_covariance: { type: 'float64[9]', default: new Array(9).fill(0) }
  },
  create: (data = {}) => ({
    header: data.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: 'imu' },
    orientation: data.orientation || { x: 0, y: 0, z: 0, w: 1 },
    orientation_covariance: data.orientation_covariance || new Array(9).fill(0),
    angular_velocity: data.angular_velocity || { x: 0, y: 0, z: 0 },
    angular_velocity_covariance: data.angular_velocity_covariance || new Array(9).fill(0),
    linear_acceleration: data.linear_acceleration || { x: 0, y: 0, z: 0 },
    linear_acceleration_covariance: data.linear_acceleration_covariance || new Array(9).fill(0)
  }),
  definition: `# IMU sensor data.
std_msgs/Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance`
};

// Self-register all message types
messageRegistry.registerMessage('sensor_msgs/msg/LaserScan', LaserScan);
messageRegistry.registerMessage('sensor_msgs/msg/Imu', Imu);

// Export all message types
export default {
  LaserScan,
  Imu
};
