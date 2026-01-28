/**
 * Node Registry - Maps package/executable names to node classes
 */

// Import node classes
import { TurtlesimNode } from './turtlesim/TurtlesimNode.js';
import { TurtleTeleopNode } from './turtlesim/TurtleTeleopNode.js';
import { TeleopTwistKeyboardNode } from './teleop_twist_keyboard/TeleopNode.js';
import { SlamNode } from './simple_slam/SlamNode.js';
import { StaticTransformPublisher } from './tf2_ros/StaticTransformPublisher.js';

/**
 * Registry of all available packages and their executables
 */
const registry = {
  turtlesim: {
    turtlesim_node: TurtlesimNode,
    turtle_teleop_key: TurtleTeleopNode
  },
  teleop_twist_keyboard: {
    teleop_twist_keyboard: TeleopTwistKeyboardNode
  },
  simple_slam: {
    slam_node: SlamNode
  },
  tf2_ros: {
    static_transform_publisher: StaticTransformPublisher
  }
};

/**
 * NodeRegistry class
 */
class NodeRegistryClass {
  constructor() {
    this.packages = registry;
  }

  /**
   * Get a node class by package and executable name
   * @param {string} packageName - Package name
   * @param {string} executableName - Executable name
   * @returns {Class|null} Node class or null if not found
   */
  getNode(packageName, executableName) {
    const pkg = this.packages[packageName];
    if (!pkg) return null;
    return pkg[executableName] || null;
  }

  /**
   * List all available packages
   * @returns {string[]} Package names
   */
  listPackages() {
    return Object.keys(this.packages);
  }

  /**
   * List executables in a package
   * @param {string} packageName - Package name
   * @returns {string[]} Executable names
   */
  listExecutables(packageName) {
    const pkg = this.packages[packageName];
    if (!pkg) return [];
    return Object.keys(pkg);
  }

  /**
   * Register a new node
   * @param {string} packageName - Package name
   * @param {string} executableName - Executable name
   * @param {Class} nodeClass - Node class
   */
  register(packageName, executableName, nodeClass) {
    if (!this.packages[packageName]) {
      this.packages[packageName] = {};
    }
    this.packages[packageName][executableName] = nodeClass;
  }

  /**
   * Check if a package exists
   * @param {string} packageName - Package name
   * @returns {boolean}
   */
  hasPackage(packageName) {
    return !!this.packages[packageName];
  }

  /**
   * Check if an executable exists
   * @param {string} packageName - Package name
   * @param {string} executableName - Executable name
   * @returns {boolean}
   */
  hasExecutable(packageName, executableName) {
    return !!this.getNode(packageName, executableName);
  }
}

export const nodeRegistry = new NodeRegistryClass();
