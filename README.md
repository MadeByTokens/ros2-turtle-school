# <img src="logo.png" width="32" height="32" alt="logo" /> ROS 2 Turtle School

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Browser-based ROS2 CLI emulator for educational purposes. Runs entirely client-side.

**Live demo:** https://madebytokens.github.io/ros2-turtle-school/

## Features

- **Full ROS2 CLI** - `ros2 topic`, `ros2 node`, `ros2 service`, `ros2 action`, `ros2 param`, `ros2 bag`
- **Turtlesim** - Classic turtle simulation with pen drawing and teleop control
- **SLAM Demo** - Simple occupancy grid mapping with lidar visualization
- **Node Graph** - Interactive visualization of nodes and topic connections (like rqt_graph)
- **Map Overlay** - Real-time occupancy grid displayed as canvas overlay with adjustable opacity
- **Multiple Terminals** - Up to 4 terminal panes with tab management
- **Teleop Control** - WASD keys work directly from terminal (W/S: forward/back, A/D: turn, Q/E: diagonal)
- **Bag Recording** - Record and playback topic messages
- **Copy/Paste Support** - Right-click context menu in terminal

## Quick Start

```bash
npm install
npm run dev
```

Open http://localhost:3000/ros2websim/

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
│   ├── EventBus.js         # App-level event bus
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

**Event Bus:**
```javascript
import { EventBus } from './core/EventBus.js';

// Subscribe
const unsubscribe = EventBus.on('my-event', (data) => {
  console.log(data);
});

// Emit
EventBus.emit('my-event', { foo: 'bar' });

// Cleanup
unsubscribe();
```

## SLAM Tutorial

Try the built-in SLAM demonstration:

```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Start SLAM node
ros2 run simple_slam slam_node

# Terminal 3: Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Click the **Map** button in the canvas toolbar to toggle the occupancy grid overlay. Use the slider to adjust opacity. Drive the turtle around to build a map of the environment.

The SLAM node:
- Subscribes to `/scan` (lidar) and `/turtle1/pose`
- Publishes occupancy grid to `/map`
- Uses log-odds Bayesian updating
- Color coding: green = free, red = occupied, blue-gray = unknown

## Testing

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
- No QoS profiles
- Lidar simulation is basic raycasting
- SLAM is educational, not production-grade

## Dependencies

- [xterm.js](https://xtermjs.org/) - Terminal emulation
- [Cytoscape.js](https://js.cytoscape.org/) - Graph visualization
- [Vite](https://vitejs.dev/) - Build tool

## License

MIT
