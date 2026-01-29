/**
 * Message loader barrel file - imports all message modules to trigger self-registration.
 * To add a new message package, simply import it here.
 */

// Standard messages
import './std_msgs.js';
import './geometry_msgs.js';

// Turtlesim messages
import './turtlesim_msgs.js';
import './turtlesim_srvs.js';
import './turtlesim_actions.js';

// TF2 messages
import './tf2_msgs.js';

// Sensor messages
import './sensor_msgs.js';

// Navigation messages
import './nav_msgs.js';

// Nav2 actions
import './nav2_actions.js';

// Re-export the registry for convenience
export { messageRegistry } from './registry.js';
