<p align="center">
  <img src="logo_readme.png" width="96" height="96" alt="ROS 2 Turtle School logo" />
</p>


# ROS 2 Turtle School

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Browser-based ROS2 CLI emulator for educational purposes. Runs entirely client-side.

**Live demo:** https://madebytokens.github.io/ros2-turtle-school/

## Features

- **Full ROS2 CLI** - `ros2 topic`, `ros2 node`, `ros2 service`, `ros2 action`, `ros2 param`, `ros2 bag`
- **Turtlesim** - Classic turtle simulation with pen drawing and teleop control
- **SLAM Demo** - Simple occupancy grid mapping with lidar visualization
- **Node Graph** - Interactive visualization of nodes and topic connections (like rqt_graph)
- **Map Overlay** - Real-time occupancy grid displayed as canvas overlay with adjustable opacity
- **Multiple Terminals** - Up to 6 terminal panes with tab management
- **Teleop Control** - WASD keys work directly from terminal (W/S: forward/back, A/D: turn, Q/E: diagonal)
- **Bag Recording** - Record and playback topic messages
- **TF2 Tools** - Query transforms with tf2_echo, tf2_monitor, view_frames
- **Nav2 Path Planning** - A* path planning with NavigateToPose action
- **Loop Closure** - Educational pose-graph loop closure detection with optional drift simulation
- **Localization Mode** - Save and localize against a pre-built map
- **QoS Profiles** - Quality of Service settings (reliability, durability, history) with named presets and verbose display
- **Dynamic Reconfigure** - Parameter validation callbacks (accept/reject with reasons)
- **Copy/Paste Support** - Right-click context menu in terminal

## Quick Start

```bash
npm install
npm run dev
```

Open http://localhost:3000/ros2-turtle-school/

## Build & Deploy

```bash
npm run build      # Output in dist/
npm run deploy     # Push to gh-pages branch
```

## Architecture

```
src/
├── app.js              # Entry point, UI wiring
├── comm/               # Communication layer (swappable)
│   ├── CommInterface.js    # Abstract interface
│   └── LocalComm.js        # In-browser pub/sub
├── core/               # ROS2 kernel
│   ├── SimDDS.js           # Topic/service/action management
│   ├── Node.js             # Base node class
│   ├── ProcessManager.js   # Node lifecycle
│   ├── ParamServer.js      # Parameter storage
│   ├── BagRecorder.js      # Topic recording
│   ├── BagPlayer.js        # Bag playback
│   ├── TFBuffer.js         # Transform tree
│   ├── Events.js           # Event name constants
│   ├── ServiceContainer.js # Dependency injection container
│   └── WorldState.js       # Obstacles, raycasting
├── msgs/               # Message definitions
│   ├── registry.js         # Message registry singleton
│   ├── loader.js           # Barrel file (imports all)
│   ├── index.js            # Public API
│   ├── geometry_msgs.js
│   ├── sensor_msgs.js
│   ├── nav_msgs.js
│   └── turtlesim_*.js
├── nodes/              # Simulated ROS2 nodes
│   ├── registry.js         # Node registry singleton
│   ├── index.js            # Barrel file (imports all)
│   ├── turtlesim/
│   ├── teleop_twist_keyboard/
│   ├── simple_slam/
│   └── tf2_ros/
├── cli/                # Command parsers
│   ├── commandRegistry.js  # Command registry singleton
│   ├── parser.js           # Main router
│   ├── builtins/           # Shell commands (clear, ls, help, etc.)
│   └── ros2_*.js           # Subcommand handlers (self-register)
└── ui/                 # UI components
    ├── Terminal.js         # xterm.js wrapper with teleop key forwarding
    ├── TerminalManager.js  # Multi-pane support
    ├── Canvas.js           # Turtle visualization + map overlay
    ├── MapCanvas.js        # Standalone occupancy grid view
    ├── Graph.js            # Cytoscape rqt_graph (nodes + topics)
    ├── Layout.js           # Responsive layout manager
    └── HelpModal.js        # Tabbed help modal
```

## Adding a New Node

Nodes self-register with the registry. Two steps:

1. Create `src/nodes/mypackage/MyNode.js`:

```javascript
import { Node } from '../../core/Node.js';
import { nodeRegistry } from '../registry.js';

export class MyNode extends Node {
  constructor(name = 'my_node', options = {}) {
    super(name, options);
  }

  onInit() {
    // Create publishers/subscribers
    this.pub = this.createPublisher('/my_topic', 'std_msgs/msg/String');
    this.createSubscription('/input', 'std_msgs/msg/String', this.handleInput.bind(this));

    // Timers
    this.createTimer(1000, () => {
      this.pub.publish({ data: 'hello' });
    });

    this.logInfo('Node started');
  }

  handleInput(msg) {
    this.logInfo(`Received: ${msg.data}`);
  }

  onShutdown() {
    // Cleanup
  }
}

// Self-register
nodeRegistry.register('mypackage', 'my_node', MyNode);
```

2. Import in `src/nodes/index.js`:

```javascript
import './mypackage/MyNode.js';
```

Now works: `ros2 run mypackage my_node`

## Adding a New Message Type

Messages self-register with the registry. Two steps:

1. Create or edit a file in `src/msgs/`:

```javascript
// src/msgs/my_msgs.js
import { messageRegistry } from './registry.js';

export const MyMessage = {
  name: 'my_msgs/msg/MyMessage',
  fields: {
    value: { type: 'float64', default: 0.0 },
    name: { type: 'string', default: '' }
  },
  create: (data = {}) => ({
    value: data.value ?? 0.0,
    name: data.name ?? ''
  }),
  definition: `float64 value\nstring name`
};

// Self-register
messageRegistry.registerMessage('my_msgs/msg/MyMessage', MyMessage);

export default { MyMessage };
```

2. Import in `src/msgs/loader.js`:

```javascript
import './my_msgs.js';
```

## Adding a CLI Command

CLI commands self-register. For ros2 subcommands:

1. Create `src/cli/ros2_mycommand.js`:

```javascript
import { commandRegistry } from './commandRegistry.js';

export async function handleRos2Mycommand(args, terminal) {
  terminal.writeln('Output here');
  terminal.finishCommand();
}

// Self-register
commandRegistry.registerRos2('mycommand', handleRos2Mycommand);

export default { handleRos2Mycommand };
```

2. Import in `src/cli/parser.js`:

```javascript
import './ros2_mycommand.js';
```

Now works: `ros2 mycommand`

## Key Patterns

**Publishing:**
```javascript
const pub = this.createPublisher('/topic', 'geometry_msgs/msg/Twist');
pub.publish({ linear: { x: 1.0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
```

**Subscribing:**
```javascript
this.createSubscription('/topic', 'std_msgs/msg/String', (msg) => {
  console.log(msg.data);
});
```

**Services:**
```javascript
// Server
this.createService('/my_service', 'std_srvs/srv/SetBool', (request) => {
  return { success: true, message: 'done' };
});

// Client (from CLI)
// ros2 service call /my_service std_srvs/srv/SetBool "{data: true}"
```

**Parameters:**
```javascript
// In constructor options
parameters: {
  my_param: 42
}

// Usage
const val = this.getParameter('my_param');
this.setParameter('my_param', 100);
```

**QoS Profiles:**
```bash
# Show QoS profiles for all publishers/subscribers on a topic
ros2 topic info /turtle1/cmd_vel --verbose

# Publish with custom QoS
ros2 topic pub /test std_msgs/msg/String "{data: 'hello'}" --qos-reliability best_effort

# Subscribe with a named QoS profile
ros2 topic echo /scan --qos-profile sensor_data

# Available profiles: default, sensor_data, services_default, parameters
# Individual flags: --qos-reliability, --qos-durability, --qos-history, --qos-depth
```

**Events (window events with constants):**
```javascript
import { Events } from './core/Events.js';

// Listen
window.addEventListener(Events.TURTLESIM_UPDATE, (event) => {
  console.log(event.detail);
});

// Dispatch
window.dispatchEvent(new CustomEvent(Events.TURTLESIM_UPDATE, {
  detail: { turtles: [], trails: [] }
}));
```

## SLAM Tutorial

### Getting Started

Open three terminals and start the required nodes:

```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Start SLAM node
ros2 run simple_slam slam_node

# Terminal 3: Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Click the **Map** button in the canvas toolbar to toggle the occupancy grid overlay. Use the slider to adjust opacity. Drive the turtle with WASD keys to build a map of the environment.

- **White cells** = Free space (lidar rays passed through)
- **Black cells** = Obstacles (lidar rays hit something)
- **Gray cells** = Unknown (not yet scanned)

### Inspect the System

Once the nodes are running, use a spare terminal to explore what SLAM is doing:

```bash
# What nodes are running?
ros2 node list

# What does the SLAM node subscribe to and publish?
ros2 node info /slam_node

# What topics are active?
ros2 topic list -t
```

### Understand the Messages

Look at the structure of the sensor data and map output:

```bash
# What does a lidar scan message look like?
ros2 interface show sensor_msgs/msg/LaserScan

# What does an occupancy grid message look like?
ros2 interface show nav_msgs/msg/OccupancyGrid

# Watch live lidar data streaming in
ros2 topic echo /scan

# Watch the occupancy grid update
ros2 topic echo /map
```

### Monitor Performance

Check how fast messages are flowing:

```bash
# Lidar publishes at ~10 Hz
ros2 topic hz /scan

# Map publishes at ~2 Hz
ros2 topic hz /map

# How much data is the lidar producing?
ros2 topic bw /scan
```

### Tune SLAM Parameters

The SLAM node exposes parameters you can change at runtime:

```bash
# See all SLAM parameters
ros2 param list /slam_node

# Check current values
ros2 param get /slam_node map_resolution
ros2 param get /slam_node hit_prob
ros2 param dump /slam_node

# Experiment: increase confidence in obstacle detection
ros2 param set /slam_node hit_prob 0.95

# Experiment: reduce false-free-space by lowering miss probability
ros2 param set /slam_node miss_prob 0.1
```

Parameters:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_resolution` | 0.1 | Meters per grid cell |
| `map_width` | 110 | Grid width in cells |
| `map_height` | 110 | Grid height in cells |
| `hit_prob` | 0.9 | Probability cell is occupied when lidar hits |
| `miss_prob` | 0.3 | Probability cell is occupied when lidar passes through |

### Record and Replay

Record a mapping session and replay it later:

```bash
# Record all SLAM-related topics
ros2 bag record -o my_slam_run /scan /turtle1/pose /map

# Drive the turtle around, then press Ctrl+C to stop recording

# Inspect what was recorded
ros2 bag info my_slam_run

# Replay the session (start a fresh SLAM node first)
ros2 bag play my_slam_run

# Replay at half speed for analysis
ros2 bag play my_slam_run --rate 0.5
```

### Use Services and Actions

Control the turtle programmatically instead of teleop:

```bash
# Clear the drawing trails
ros2 service call /clear std_srvs/srv/Empty

# Rotate the turtle to a specific angle
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

# Spawn a second turtle
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 8.0, theta: 0.0, name: 'turtle2'}"
```

### How It Works

The SLAM node uses log-odds Bayesian updating:
- Subscribes to `/scan` (lidar) and `/turtle1/pose` (position)
- Traces each lidar ray through the occupancy grid
- Cells the ray passes through are marked as more likely free
- Cells where the ray terminates are marked as more likely occupied
- Publishes the occupancy grid to `/map` at 2 Hz

## Nav2 Path Planning Tutorial

### Prerequisites

Complete the SLAM tutorial above first — you need a running turtlesim, SLAM node, and a partially explored map.

### Start the Navigator

Open a new terminal and launch the Nav2 path planner:

```bash
ros2 run nav2_simple_navigator navigator_node
```

The navigator subscribes to `/map` (from SLAM) and `/turtle1/pose`, and creates a `/navigate_to_pose` action server.

### Send a Navigation Goal

Send the turtle to a target coordinate:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 8.0, y: 8.0}}}"

# With live feedback showing distance remaining:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 2.0, y: 9.0}}}" --feedback
```

The navigator plans a path using A* on the occupancy grid, publishes it to `/plan`, and drives the turtle along the path.

### Monitor and Inspect

```bash
# See the planned path
ros2 topic echo /plan

# Check the navigator node
ros2 node info /navigator_node

# See all available actions
ros2 action list -t
```

### Tune Navigator Parameters

```bash
# How close the turtle needs to get to the goal (meters)
ros2 param set /navigator_node goal_tolerance 0.5

# Maximum driving speed (m/s)
ros2 param set /navigator_node max_speed 2.0

# Occupancy threshold — cells above this value are treated as obstacles (0-100)
ros2 param set /navigator_node obstacle_threshold 30
```

Parameters:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `goal_tolerance` | 0.3 | Distance (m) to consider goal reached |
| `max_speed` | 1.5 | Maximum linear speed (m/s) |
| `max_angular_speed` | 2.5 | Maximum angular speed (rad/s) |
| `obstacle_threshold` | 50 | Cells with occupancy >= this are obstacles |

### Loop Closure & Odometry Drift

Explore why loop closure matters by enabling simulated odometry drift:

```bash
# Enable drift simulation — the map will gradually distort
ros2 param set /slam_node drift_enabled true
ros2 param set /slam_node drift_rate 0.02

# Watch for loop closure detection events
ros2 topic echo /loop_closures

# Drive the turtle in a loop — when it revisits a location,
# the SLAM node detects the closure and corrects the drift
```

Loop closure parameters:
| Parameter | Default | Description |
|-----------|---------|-------------|
| `loop_closure_enabled` | true | Enable loop closure detection |
| `loop_closure_distance` | 0.5 | Distance threshold to trigger closure (m) |
| `loop_closure_min_travel` | 3.0 | Minimum travel before checking (m) |
| `loop_closure_cooldown` | 5000 | Milliseconds between detections |
| `drift_enabled` | false | Enable odometry drift simulation |
| `drift_rate` | 0.005 | Drift magnitude per meter traveled |
| `drift_max` | 0.3 | Maximum cumulative drift (m) |

### Localization Against a Pre-built Map

Save a map you've built and switch to localization mode (no new mapping):

```bash
# Save the current map
ros2 service call /slam/save_map std_srvs/srv/Empty

# Switch to localization mode — the map stops updating
ros2 param set /slam_node localization_mode true

# Switch back to mapping mode
ros2 param set /slam_node localization_mode false

# Reload the saved map
ros2 service call /slam/load_map std_srvs/srv/Empty
```

### TF2 Frame Queries

Explore coordinate transforms between frames:

```bash
# Publish a static transform from world to base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_link

# Continuously print the transform between two frames
ros2 run tf2_ros tf2_echo world base_link

# Print the full frame tree
ros2 run tf2_ros view_frames

# Monitor all TF broadcasts and their rates
ros2 run tf2_ros tf2_monitor
```

### How A* Path Planning Works

The navigator uses the A* algorithm on the SLAM occupancy grid:
1. Converts start/goal world positions to grid cells
2. Searches the grid using A* with 8-connected neighbors (diagonal movement allowed)
3. Avoids cells with occupancy >= `obstacle_threshold`
4. Reconstructs the path and publishes it to `/plan`
5. Follows the path using a proportional controller publishing to `/turtle1/cmd_vel`

## Testing

### Automated Tests

The project uses [Vitest](https://vitest.dev/) for automated testing.

```bash
npm test              # Run tests in watch mode
npm test -- --run     # Run tests once
npm run test:coverage # Run tests with coverage report
```

Test suite covers the core ROS 2 emulation layer (386 tests across 11 files):

| Test File | Module | Tests |
|-----------|--------|-------|
| `src/cli/ros2_commands.test.js` | CLI command handlers (all ros2 subcommands) | 97 |
| `src/core/WorldState.test.js` | Collision, raycasting, lidar | 50 |
| `src/comm/LocalComm.test.js` | Pub/sub, services, actions, QoS storage | 46 |
| `src/utils/qos.test.js` | QoS profiles, parsing, formatting | 35 |
| `src/cli/messageParser.test.js` | YAML-like message parsing | 33 |
| `src/core/Node.test.js` | Parameters, timers, lifecycle | 33 |
| `src/msgs/registry.test.js` | Message/service/action registry | 31 |
| `src/core/ProcessManager.test.js` | Process spawn, kill, queries | 28 |
| `src/core/Node.dynreconfig.test.js` | Dynamic reconfigure callbacks | 16 |
| `src/nodes/nav2_simple_navigator/NavigatorNode.test.js` | Nav2 A* path planning | 9 |
| `src/nodes/tf2_ros/TF2Nodes.test.js` | TF2 echo, monitor, view_frames | 8 |

### Runtime Logging

Open browser console for debug output. All nodes log via `LogManager`.

```javascript
// In node
this.logDebug('verbose');
this.logInfo('normal');
this.logWarn('warning');
this.logError('error');
```

View logs: run `rqt_console` in terminal.

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Ctrl+C` | Interrupt running command |
| `Ctrl+L` | Clear terminal |
| `↑` / `↓` | Command history |
| Right-click | Copy/Paste/Clear menu |

**Teleop controls** (when teleop node is running):
| Key | Action |
|-----|--------|
| `W` / `S` | Forward / Backward |
| `A` / `D` | Turn left / Turn right |
| `Q` / `E` | Diagonal (forward + turn) |
| `X` | Stop |

## Limitations

- No real networking (single browser context)
- Simplified message validation
- QoS profiles are stored and displayed but not enforced at the transport level
- Lidar simulation is basic raycasting
- SLAM is educational, not production-grade
- Nav2 path planning uses simple A* (no costmaps or DWA local planner)
- Loop closure is proximity-based (no scan matching or graph optimization)

## Dependencies

- [xterm.js](https://xtermjs.org/) - Terminal emulation
- [Cytoscape.js](https://js.cytoscape.org/) - Graph visualization
- [Vite](https://vitejs.dev/) - Build tool
- [Vitest](https://vitest.dev/) - Test framework (dev)

## License

MIT
