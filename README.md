# ROS2 Web Emulator

Browser-based ROS2 CLI emulator for educational purposes. Runs entirely client-side.

**Live demo:** https://madebytokens.github.io/ros2websim/

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
│   └── WorldState.js       # Obstacles, raycasting
├── msgs/               # Message definitions
│   ├── index.js            # Registry
│   ├── geometry_msgs.js
│   ├── sensor_msgs.js
│   ├── nav_msgs.js
│   └── turtlesim_*.js
├── nodes/              # Simulated ROS2 nodes
│   ├── registry.js         # Package/executable mapping
│   ├── turtlesim/
│   ├── teleop_twist_keyboard/
│   ├── simple_slam/
│   └── tf2_ros/
├── cli/                # Command parsers
│   ├── parser.js           # Main router
│   └── ros2_*.js           # Subcommand handlers
└── ui/                 # UI components
    ├── Terminal.js         # xterm.js wrapper
    ├── TerminalManager.js  # Multi-pane support
    ├── Canvas.js           # Turtle visualization
    ├── MapCanvas.js        # Occupancy grid
    └── Graph.js            # Cytoscape rqt_graph
```

## Adding a New Node

1. Create `src/nodes/mypackage/MyNode.js`:

```javascript
import { Node } from '../../core/Node.js';

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
```

2. Register in `src/nodes/registry.js`:

```javascript
import { MyNode } from './mypackage/MyNode.js';

const registry = {
  // ...
  mypackage: {
    my_node: MyNode
  }
};
```

3. Now works: `ros2 run mypackage my_node`

## Adding a New Message Type

1. Add to appropriate file in `src/msgs/`:

```javascript
// src/msgs/my_msgs.js
export default {
  MyMessage: {
    fields: {
      value: { type: 'float64', default: 0.0 },
      name: { type: 'string', default: '' }
    },
    create: (data = {}) => ({
      value: data.value ?? 0.0,
      name: data.name ?? ''
    }),
    definition: `float64 value\nstring name`
  }
};
```

2. Register in `src/msgs/index.js`:

```javascript
import my_msgs from './my_msgs.js';

const messages = {
  // ...
  'my_msgs/msg/MyMessage': my_msgs.MyMessage
};
```

## Adding a CLI Command

Edit `src/cli/parser.js` or create a new handler file:

```javascript
// In parser.js switch statement
case 'mycommand':
  terminal.writeln('Output here');
  terminal.finishCommand();
  break;
```

For ros2 subcommands, create `src/cli/ros2_mycommand.js` and import in `parser.js`.

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
