/**
 * Node barrel file - imports all node modules to trigger self-registration.
 * To add a new node, simply import it here.
 */

// Turtlesim package
import './turtlesim/TurtlesimNode.js';
import './turtlesim/TurtleTeleopNode.js';

// Teleop twist keyboard package
import './teleop_twist_keyboard/TeleopNode.js';

// Simple SLAM package
import './simple_slam/SlamNode.js';

// TF2 ROS package
import './tf2_ros/StaticTransformPublisher.js';

// Re-export the registry for convenience
export { nodeRegistry } from './registry.js';
