/**
 * CLI loader barrel file - imports all command handlers to trigger self-registration.
 * To add a new command, simply import it here.
 */

// ROS2 subcommands
import './ros2_run.js';
import './ros2_node.js';
import './ros2_topic.js';
import './ros2_service.js';
import './ros2_action.js';
import './ros2_param.js';
import './ros2_bag.js';
import './ros2_interface.js';
import './ros2_pkg.js';

// Re-export the command registry
export { commandRegistry } from './commandRegistry.js';

// Re-export the parser
export { parseCommand } from './parser.js';
