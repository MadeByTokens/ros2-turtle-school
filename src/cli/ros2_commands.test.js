import { describe, it, expect, beforeEach, vi, afterEach } from 'vitest';

// ── helpers ──────────────────────────────────────────────────────────
// Mock terminal that captures writeln output
function createMockTerminal() {
  const lines = [];
  return {
    lines,
    writeln(text) { lines.push(text); },
    write(text) { lines.push(text); },
    finishCommand: vi.fn(),
    addSubscription: vi.fn(),
    addBagRecorder: vi.fn(),
  };
}

// ── SimDDS mock ──────────────────────────────────────────────────────
vi.mock('../core/SimDDS.js', () => {
  const state = {
    nodes: [],
    nodeInfo: {},
    topics: [],
    topicInfo: {},
    services: [],
    serviceInfo: {},
    actions: [],
    actionInfo: {},
    subscribeCb: null,
    callServiceResult: {},
    sendActionGoalResult: {},
  };

  return {
    SimDDS: {
      _state: state,
      getNodes: () => state.nodes,
      getNodeInfo: (id) => state.nodeInfo[id],
      getTopics: () => state.topics,
      getTopicInfo: (name) => state.topicInfo[name],
      getServices: () => state.services,
      getServiceInfo: (name) => state.serviceInfo[name],
      getActions: () => state.actions,
      getActionInfo: (name) => state.actionInfo[name],
      subscribe: vi.fn((topic, type, cb) => {
        state.subscribeCb = cb;
        return 'sub_1';
      }),
      unsubscribe: vi.fn(),
      publish: vi.fn(),
      callService: vi.fn(async () => state.callServiceResult),
      sendActionGoal: vi.fn(async (name, type, goal, fbCb) => {
        if (fbCb) fbCb({ remaining: 0.5 });
        return state.sendActionGoalResult;
      }),
    },
  };
});

// ── ProcessManager mock ──────────────────────────────────────────────
vi.mock('../core/ProcessManager.js', () => {
  const state = { processes: {} };
  return {
    ProcessManager: {
      _state: state,
      getProcessByNodeName: (name) => state.processes[name] || null,
    },
  };
});

// ── msgs/index mock ──────────────────────────────────────────────────
vi.mock('../msgs/index.js', () => ({
  getMessage: vi.fn((type) => {
    if (type === 'geometry_msgs/msg/Twist') {
      return {
        name: 'geometry_msgs/msg/Twist',
        create: (data = {}) => ({
          linear: { x: data.linear?.x ?? 0, y: data.linear?.y ?? 0, z: data.linear?.z ?? 0 },
          angular: { x: data.angular?.x ?? 0, y: data.angular?.y ?? 0, z: data.angular?.z ?? 0 },
        }),
      };
    }
    return null;
  }),
  findMessages: vi.fn(() => []),
  getService: vi.fn((type) => {
    if (type === 'std_srvs/srv/Empty') {
      return {
        name: 'std_srvs/srv/Empty',
        createRequest: () => ({}),
      };
    }
    if (type === 'turtlesim/srv/Spawn') {
      return {
        name: 'turtlesim/srv/Spawn',
        createRequest: (data = {}) => ({
          x: data.x ?? 0, y: data.y ?? 0, theta: data.theta ?? 0, name: data.name ?? '',
        }),
      };
    }
    return null;
  }),
  findServices: vi.fn(() => []),
  getAction: vi.fn((type) => {
    if (type === 'turtlesim/action/RotateAbsolute') {
      return {
        name: 'turtlesim/action/RotateAbsolute',
        createGoal: (data = {}) => ({ theta: data.theta ?? 0 }),
      };
    }
    return null;
  }),
  findActions: vi.fn(() => []),
  getInterface: vi.fn(() => null),
  getDefinition: vi.fn((type) => {
    if (type === 'geometry_msgs/msg/Twist') {
      return 'geometry_msgs/Vector3 linear\n  float64 x\n  float64 y\n  float64 z\ngeometry_msgs/Vector3 angular\n  float64 x\n  float64 y\n  float64 z';
    }
    if (type === 'std_srvs/srv/Empty') {
      return '---';
    }
    if (type === 'turtlesim/srv/Spawn') {
      return 'float32 x\nfloat32 y\nfloat32 theta\nstring name\n---\nstring name';
    }
    if (type === 'turtlesim/action/RotateAbsolute') {
      return 'float32 theta\n---\nfloat32 delta\n---\nfloat32 remaining';
    }
    return null;
  }),
  listMessages: vi.fn(() => ['geometry_msgs/msg/Twist', 'std_msgs/msg/String']),
  listServices: vi.fn(() => ['std_srvs/srv/Empty', 'turtlesim/srv/Spawn']),
  listActions: vi.fn(() => ['turtlesim/action/RotateAbsolute']),
}));

// ── messageParser mock ───────────────────────────────────────────────
vi.mock('./messageParser.js', () => ({
  parseMessage: vi.fn((str) => {
    if (!str) return {};
    try {
      // Minimal YAML-ish parse for tests
      const clean = str.replace(/'/g, '"');
      return JSON.parse(clean);
    } catch {
      return {};
    }
  }),
  formatMessage: vi.fn((msg) => {
    if (msg === null || msg === undefined) return 'null';
    if (typeof msg !== 'object') return String(msg);
    return Object.entries(msg)
      .map(([k, v]) => typeof v === 'object' && v !== null
        ? `${k}:\n${Object.entries(v).map(([k2, v2]) => `  ${k2}: ${v2}`).join('\n')}`
        : `${k}: ${v}`)
      .join('\n');
  }),
}));

// ── BagRecorder / BagStorage / BagPlayer mock ────────────────────────
vi.mock('../core/BagRecorder.js', () => {
  const _bags = {};
  class MockBagRecorder {
    constructor(name, topics) {
      this.name = name;
      this.topics = topics;
    }
    start() {}
    stop() {}
    getInfo() {
      return {
        name: this.name,
        duration: 10.5,
        messageCount: 42,
        topics: this.topics,
        topicCounts: Object.fromEntries(this.topics.map(t => [t, 14])),
      };
    }
  }
  return {
    BagRecorder: MockBagRecorder,
    BagStorage: {
      _bags,
      store: vi.fn((recorder) => { _bags[recorder.name] = recorder; }),
      get: vi.fn((name) => _bags[name] || null),
      list: vi.fn(() => Object.keys(_bags)),
    },
  };
});

vi.mock('../core/BagPlayer.js', () => {
  class MockBagPlayer {
    constructor() {
      this.onComplete = null;
      this.onMessage = null;
    }
    play() {
      if (this.onComplete) this.onComplete();
    }
  }
  return { BagPlayer: MockBagPlayer };
});

// ── commandRegistry mock ─────────────────────────────────────────────
vi.mock('./commandRegistry.js', () => ({
  commandRegistry: {
    registerRos2: vi.fn(),
  },
}));

// ── Imports (after mocks) ────────────────────────────────────────────
import { SimDDS } from '../core/SimDDS.js';
import { ProcessManager } from '../core/ProcessManager.js';
import { handleRos2Topic } from './ros2_topic.js';
import { handleRos2Node } from './ros2_node.js';
import { handleRos2Service } from './ros2_service.js';
import { handleRos2Param } from './ros2_param.js';
import { handleRos2Action } from './ros2_action.js';
import { handleRos2Interface } from './ros2_interface.js';
import { handleRos2Bag } from './ros2_bag.js';
import { BagStorage } from '../core/BagRecorder.js';

// ── Reset state before each test ─────────────────────────────────────
beforeEach(() => {
  const s = SimDDS._state;
  s.nodes = [];
  s.nodeInfo = {};
  s.topics = [];
  s.topicInfo = {};
  s.services = [];
  s.serviceInfo = {};
  s.actions = [];
  s.actionInfo = {};
  s.subscribeCb = null;
  s.callServiceResult = {};
  s.sendActionGoalResult = {};
  ProcessManager._state.processes = {};
  // Clear bag storage
  for (const key of Object.keys(BagStorage._bags)) {
    delete BagStorage._bags[key];
  }
  vi.clearAllMocks();
});

afterEach(() => {
  vi.restoreAllMocks();
});

// =====================================================================
// Tutorial 1: Understanding Nodes
// =====================================================================
describe('ros2 node (Tutorial: Understanding Nodes)', () => {
  describe('ros2 node list', () => {
    it('lists running nodes sorted alphabetically', async () => {
      SimDDS._state.nodes = ['/turtlesim', '/teleop_turtle'];
      const t = createMockTerminal();
      await handleRos2Node(['list'], t);

      expect(t.lines).toEqual(['/teleop_turtle', '/turtlesim']);
    });

    it('shows message when no nodes are running', async () => {
      const t = createMockTerminal();
      await handleRos2Node(['list'], t);

      expect(t.lines[0]).toContain('No nodes');
    });
  });

  describe('ros2 node info', () => {
    it('shows node info with subscribers, publishers, services, and actions', async () => {
      SimDDS._state.nodeInfo['/turtlesim'] = { name: '/turtlesim' };
      ProcessManager._state.processes['/turtlesim'] = {
        node: {
          subscriptions: [
            { topic: '/turtle1/cmd_vel', msgType: 'geometry_msgs/msg/Twist' },
          ],
          publishers: [
            { topic: '/turtle1/pose', msgType: 'turtlesim/msg/Pose' },
            { topic: '/turtle1/color_sensor', msgType: 'turtlesim/msg/Color' },
          ],
          services: [
            { name: '/clear', type: 'std_srvs/srv/Empty' },
            { name: '/spawn', type: 'turtlesim/srv/Spawn' },
          ],
          actionServers: [
            { name: '/turtle1/rotate_absolute', type: 'turtlesim/action/RotateAbsolute' },
          ],
        },
      };

      const t = createMockTerminal();
      await handleRos2Node(['info', '/turtlesim'], t);

      // Node name at column 0
      expect(t.lines[0]).toBe('/turtlesim');

      // Section headers at 2-space indent
      expect(t.lines).toContain('  Subscribers:');
      expect(t.lines).toContain('  Publishers:');
      expect(t.lines).toContain('  Service Servers:');
      expect(t.lines).toContain('  Service Clients:');
      expect(t.lines).toContain('  Action Servers:');
      expect(t.lines).toContain('  Action Clients:');

      // Items at 4-space indent
      expect(t.lines).toContain('    /turtle1/cmd_vel: geometry_msgs/msg/Twist');
      expect(t.lines).toContain('    /turtle1/pose: turtlesim/msg/Pose');
      expect(t.lines).toContain('    /clear: std_srvs/srv/Empty');
      expect(t.lines).toContain('    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute');
    });

    it('shows empty sections without (none) text (matches real ros2 CLI)', async () => {
      SimDDS._state.nodeInfo['/teleop_turtle'] = { name: '/teleop_turtle' };
      ProcessManager._state.processes['/teleop_turtle'] = {
        node: {
          subscriptions: [],
          publishers: [{ topic: '/turtle1/cmd_vel', msgType: 'geometry_msgs/msg/Twist' }],
          services: [],
          actionServers: [],
        },
      };

      const t = createMockTerminal();
      await handleRos2Node(['info', '/teleop_turtle'], t);

      // Empty sections should NOT contain '(none)'
      expect(t.lines.join('\n')).not.toContain('(none)');

      // Empty Subscribers: header should be immediately followed by Publishers: header
      const subIdx = t.lines.indexOf('  Subscribers:');
      expect(t.lines[subIdx + 1]).toBe('  Publishers:');
    });

    it('adds leading slash if not present', async () => {
      SimDDS._state.nodeInfo['/turtlesim'] = { name: '/turtlesim' };
      ProcessManager._state.processes['/turtlesim'] = {
        node: {
          subscriptions: [], publishers: [], services: [], actionServers: [],
        },
      };

      const t = createMockTerminal();
      await handleRos2Node(['info', 'turtlesim'], t);

      expect(t.lines[0]).toBe('/turtlesim');
    });

    it('shows error for unknown node', async () => {
      const t = createMockTerminal();
      await handleRos2Node(['info', '/nonexistent'], t);

      expect(t.lines[0]).toContain('not found');
    });
  });
});

// =====================================================================
// Tutorial 2: Understanding Topics
// =====================================================================
describe('ros2 topic (Tutorial: Understanding Topics)', () => {
  describe('ros2 topic list', () => {
    it('lists topics without types', async () => {
      SimDDS._state.topics = [
        { name: '/turtle1/cmd_vel', type: 'geometry_msgs/msg/Twist' },
        { name: '/turtle1/pose', type: 'turtlesim/msg/Pose' },
      ];
      const t = createMockTerminal();
      await handleRos2Topic(['list'], t);

      expect(t.lines).toContain('/turtle1/cmd_vel');
      expect(t.lines).toContain('/turtle1/pose');
      // No brackets without -t flag
      expect(t.lines.join('')).not.toContain('[');
    });

    it('lists topics with types when -t flag is passed', async () => {
      SimDDS._state.topics = [
        { name: '/turtle1/cmd_vel', type: 'geometry_msgs/msg/Twist' },
      ];
      const t = createMockTerminal();
      await handleRos2Topic(['list', '-t'], t);

      expect(t.lines).toContain('/turtle1/cmd_vel [geometry_msgs/msg/Twist]');
    });

    it('lists topics sorted alphabetically', async () => {
      SimDDS._state.topics = [
        { name: '/z_topic', type: 'std_msgs/msg/String' },
        { name: '/a_topic', type: 'std_msgs/msg/String' },
      ];
      const t = createMockTerminal();
      await handleRos2Topic(['list'], t);

      expect(t.lines[0]).toBe('/a_topic');
      expect(t.lines[1]).toBe('/z_topic');
    });
  });

  describe('ros2 topic echo', () => {
    it('subscribes to a topic and prints messages when received', async () => {
      SimDDS._state.topicInfo['/turtle1/cmd_vel'] = {
        name: '/turtle1/cmd_vel',
        type: 'geometry_msgs/msg/Twist',
      };

      const t = createMockTerminal();
      await handleRos2Topic(['echo', '/turtle1/cmd_vel'], t);

      // Should have called subscribe
      expect(SimDDS.subscribe).toHaveBeenCalledWith(
        '/turtle1/cmd_vel',
        'geometry_msgs/msg/Twist',
        expect.any(Function),
      );

      // Simulate receiving a message
      const cb = SimDDS.subscribe.mock.calls[0][2];
      cb({ linear: { x: 2.0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 1.8 } });

      // Should print separator and formatted message
      expect(t.lines).toContain('---');

      // Should NOT have called finishCommand (runs until Ctrl+C)
      expect(t.finishCommand).not.toHaveBeenCalled();

      // Should register cleanup subscription
      expect(t.addSubscription).toHaveBeenCalled();
    });

    it('waits for topic when it does not exist yet', async () => {
      // Topic does not exist
      const t = createMockTerminal();
      await handleRos2Topic(['echo', '/future_topic'], t);

      expect(t.lines.join(' ')).toContain('does not exist yet');
      expect(t.addSubscription).toHaveBeenCalled();
    });

    it('shows usage when no topic name given', async () => {
      const t = createMockTerminal();
      await handleRos2Topic(['echo'], t);

      expect(t.lines[0]).toContain('usage:');
      expect(t.finishCommand).toHaveBeenCalled();
    });
  });

  describe('ros2 topic info', () => {
    it('shows type, publisher count, and subscription count', async () => {
      SimDDS._state.topicInfo['/turtle1/cmd_vel'] = {
        name: '/turtle1/cmd_vel',
        type: 'geometry_msgs/msg/Twist',
        publishers: [{ nodeId: '/teleop_turtle' }],
        subscribers: [{ nodeId: '/turtlesim' }],
      };

      const t = createMockTerminal();
      await handleRos2Topic(['info', '/turtle1/cmd_vel'], t);

      expect(t.lines).toContain('Type: geometry_msgs/msg/Twist');
      expect(t.lines).toContain('Publisher count: 1');
      expect(t.lines).toContain('Subscription count: 1');
    });

    it('shows error for unknown topic', async () => {
      const t = createMockTerminal();
      await handleRos2Topic(['info', '/nonexistent'], t);

      expect(t.lines[0]).toContain('not found');
    });
  });

  describe('ros2 topic pub', () => {
    it('publishes a message to a topic', async () => {
      const t = createMockTerminal();
      await handleRos2Topic([
        'pub', '--once', '/turtle1/cmd_vel',
        'geometry_msgs/msg/Twist',
        '{"linear": {"x": 2.0}, "angular": {"z": 1.8}}',
      ], t);

      expect(SimDDS.publish).toHaveBeenCalledWith(
        '/turtle1/cmd_vel',
        'geometry_msgs/msg/Twist',
        expect.objectContaining({
          linear: expect.objectContaining({ x: 2.0 }),
          angular: expect.objectContaining({ z: 1.8 }),
        }),
      );
      expect(t.lines.join('\n')).toContain('publishing #1');
      expect(t.finishCommand).toHaveBeenCalled();
    });

    it('shows error for unknown message type', async () => {
      const t = createMockTerminal();
      await handleRos2Topic([
        'pub', '--once', '/topic',
        'fake_msgs/msg/DoesNotExist', '{}',
      ], t);

      expect(t.lines[0]).toContain('Unknown message type');
      expect(SimDDS.publish).not.toHaveBeenCalled();
    });

    it('shows usage when missing arguments', async () => {
      const t = createMockTerminal();
      await handleRos2Topic(['pub'], t);

      expect(t.lines[0]).toContain('usage:');
    });
  });

  describe('ros2 topic type', () => {
    it('shows the type of a topic', async () => {
      SimDDS._state.topicInfo['/turtle1/cmd_vel'] = {
        name: '/turtle1/cmd_vel',
        type: 'geometry_msgs/msg/Twist',
      };

      const t = createMockTerminal();
      await handleRos2Topic(['type', '/turtle1/cmd_vel'], t);

      expect(t.lines).toContain('geometry_msgs/msg/Twist');
    });
  });

  describe('ros2 topic find', () => {
    it('finds topics by message type', async () => {
      SimDDS._state.topics = [
        { name: '/turtle1/cmd_vel', type: 'geometry_msgs/msg/Twist' },
        { name: '/turtle1/pose', type: 'turtlesim/msg/Pose' },
      ];

      const t = createMockTerminal();
      await handleRos2Topic(['find', 'geometry_msgs/msg/Twist'], t);

      expect(t.lines).toContain('/turtle1/cmd_vel');
      expect(t.lines).not.toContain('/turtle1/pose');
    });
  });

  describe('ros2 topic hz', () => {
    it('subscribes and reports rate when messages arrive', async () => {
      SimDDS._state.topicInfo['/turtle1/pose'] = {
        name: '/turtle1/pose',
        type: 'turtlesim/msg/Pose',
      };

      const t = createMockTerminal();
      await handleRos2Topic(['hz', '/turtle1/pose'], t);

      expect(SimDDS.subscribe).toHaveBeenCalled();
      expect(t.addSubscription).toHaveBeenCalled();

      // Should not finish (runs until Ctrl+C)
      expect(t.finishCommand).not.toHaveBeenCalled();
    });

    it('shows waiting message for unknown topic', async () => {
      const t = createMockTerminal();
      await handleRos2Topic(['hz', '/nonexistent'], t);

      expect(t.lines[0]).toContain('Waiting');
    });
  });

  describe('ros2 topic bw', () => {
    it('subscribes and reports bandwidth when messages arrive', async () => {
      SimDDS._state.topicInfo['/turtle1/pose'] = {
        name: '/turtle1/pose',
        type: 'turtlesim/msg/Pose',
      };

      const t = createMockTerminal();
      await handleRos2Topic(['bw', '/turtle1/pose'], t);

      expect(SimDDS.subscribe).toHaveBeenCalled();
      expect(t.addSubscription).toHaveBeenCalled();
      expect(t.finishCommand).not.toHaveBeenCalled();
    });
  });
});

// =====================================================================
// Tutorial 3: Understanding Services
// =====================================================================
describe('ros2 service (Tutorial: Understanding Services)', () => {
  describe('ros2 service list', () => {
    it('lists services without types', async () => {
      SimDDS._state.services = [
        { name: '/clear', type: 'std_srvs/srv/Empty' },
        { name: '/spawn', type: 'turtlesim/srv/Spawn' },
      ];

      const t = createMockTerminal();
      await handleRos2Service(['list'], t);

      expect(t.lines).toContain('/clear');
      expect(t.lines).toContain('/spawn');
      expect(t.lines.join('')).not.toContain('[');
    });

    it('lists services with types when -t flag is passed', async () => {
      SimDDS._state.services = [
        { name: '/clear', type: 'std_srvs/srv/Empty' },
      ];

      const t = createMockTerminal();
      await handleRos2Service(['list', '-t'], t);

      expect(t.lines).toContain('/clear [std_srvs/srv/Empty]');
    });

    it('lists services sorted alphabetically', async () => {
      SimDDS._state.services = [
        { name: '/spawn', type: 'turtlesim/srv/Spawn' },
        { name: '/clear', type: 'std_srvs/srv/Empty' },
      ];

      const t = createMockTerminal();
      await handleRos2Service(['list'], t);

      expect(t.lines[0]).toBe('/clear');
      expect(t.lines[1]).toBe('/spawn');
    });
  });

  describe('ros2 service type', () => {
    it('shows the type of a service', async () => {
      SimDDS._state.serviceInfo['/clear'] = {
        name: '/clear',
        type: 'std_srvs/srv/Empty',
        node: '/turtlesim',
      };

      const t = createMockTerminal();
      await handleRos2Service(['type', '/clear'], t);

      expect(t.lines).toContain('std_srvs/srv/Empty');
    });
  });

  describe('ros2 service info', () => {
    it('shows type and node for a service', async () => {
      SimDDS._state.serviceInfo['/clear'] = {
        name: '/clear',
        type: 'std_srvs/srv/Empty',
        node: '/turtlesim',
      };

      const t = createMockTerminal();
      await handleRos2Service(['info', '/clear'], t);

      expect(t.lines).toContain('Type: std_srvs/srv/Empty');
      expect(t.lines).toContain('Node: /turtlesim');
    });
  });

  describe('ros2 service find', () => {
    it('finds services by type', async () => {
      SimDDS._state.services = [
        { name: '/clear', type: 'std_srvs/srv/Empty' },
        { name: '/spawn', type: 'turtlesim/srv/Spawn' },
      ];

      const t = createMockTerminal();
      await handleRos2Service(['find', 'std_srvs/srv/Empty'], t);

      expect(t.lines).toContain('/clear');
      expect(t.lines).not.toContain('/spawn');
    });
  });

  describe('ros2 service call', () => {
    it('calls a service and shows the response', async () => {
      SimDDS._state.serviceInfo['/clear'] = {
        name: '/clear',
        type: 'std_srvs/srv/Empty',
        node: '/turtlesim',
      };
      SimDDS._state.callServiceResult = { success: true };

      const t = createMockTerminal();
      await handleRos2Service(['call', '/clear', 'std_srvs/srv/Empty'], t);

      expect(SimDDS.callService).toHaveBeenCalledWith(
        '/clear',
        'std_srvs/srv/Empty',
        expect.any(Object),
      );
      expect(t.lines.join('\n')).toContain('response:');
    });

    it('calls spawn service with request data', async () => {
      SimDDS._state.serviceInfo['/spawn'] = {
        name: '/spawn',
        type: 'turtlesim/srv/Spawn',
        node: '/turtlesim',
      };
      SimDDS._state.callServiceResult = { name: 'turtle2' };

      const t = createMockTerminal();
      await handleRos2Service([
        'call', '/spawn', 'turtlesim/srv/Spawn',
        '{"x": 2, "y": 2, "theta": 0.2, "name": ""}',
      ], t);

      expect(SimDDS.callService).toHaveBeenCalled();
      expect(t.lines.join('\n')).toContain('response:');
    });

    it('shows error for unknown service type', async () => {
      const t = createMockTerminal();
      await handleRos2Service(['call', '/foo', 'fake_srvs/srv/Nope'], t);

      expect(t.lines[0]).toContain('Unknown service type');
    });

    it('shows error when service is not available', async () => {
      // Service type exists but service is not registered
      const t = createMockTerminal();
      await handleRos2Service(['call', '/missing', 'std_srvs/srv/Empty'], t);

      expect(t.lines[0]).toContain('not available');
    });
  });
});

// =====================================================================
// Tutorial 4: Understanding Parameters
// =====================================================================
describe('ros2 param (Tutorial: Understanding Parameters)', () => {
  const setupTurtlesimNode = () => {
    SimDDS._state.nodes = ['/turtlesim'];
    ProcessManager._state.processes['/turtlesim'] = {
      node: {
        getParameterNames: () => ['background_r', 'background_g', 'background_b', 'use_sim_time'],
        getParameter: (name) => {
          const params = { background_r: 69, background_g: 86, background_b: 255, use_sim_time: false };
          return params[name];
        },
        setParameter: vi.fn(() => true),
      },
    };
  };

  describe('ros2 param list', () => {
    it('lists all parameters for all nodes', async () => {
      setupTurtlesimNode();

      const t = createMockTerminal();
      await handleRos2Param(['list'], t);

      expect(t.lines).toContain('/turtlesim:');
      expect(t.lines).toContain('  background_b');
      expect(t.lines).toContain('  background_g');
      expect(t.lines).toContain('  background_r');
      expect(t.lines).toContain('  use_sim_time');
    });

    it('lists parameters for a specific node', async () => {
      setupTurtlesimNode();

      const t = createMockTerminal();
      await handleRos2Param(['list', '/turtlesim'], t);

      expect(t.lines[0]).toBe('/turtlesim:');
      // Parameters should be indented with 2 spaces
      for (let i = 1; i < t.lines.length; i++) {
        expect(t.lines[i]).toMatch(/^  \S/);
      }
    });

    it('shows error for unknown node', async () => {
      const t = createMockTerminal();
      await handleRos2Param(['list', '/nonexistent'], t);

      expect(t.lines[0]).toContain('not found');
    });
  });

  describe('ros2 param get', () => {
    it('shows integer parameter with type label', async () => {
      setupTurtlesimNode();

      const t = createMockTerminal();
      await handleRos2Param(['get', '/turtlesim', 'background_g'], t);

      expect(t.lines[0]).toBe('Integer value is: 86');
    });

    it('shows boolean parameter with type label', async () => {
      setupTurtlesimNode();

      const t = createMockTerminal();
      await handleRos2Param(['get', '/turtlesim', 'use_sim_time'], t);

      expect(t.lines[0]).toBe('Boolean value is: false');
    });

    it('shows error for unknown parameter', async () => {
      setupTurtlesimNode();
      // Override to return undefined for unknown param
      ProcessManager._state.processes['/turtlesim'].node.getParameter = (name) => {
        if (name === 'nonexistent') return undefined;
        return 42;
      };

      const t = createMockTerminal();
      await handleRos2Param(['get', '/turtlesim', 'nonexistent'], t);

      expect(t.lines[0]).toContain('not found');
    });
  });

  describe('ros2 param set', () => {
    it('sets a parameter value', async () => {
      setupTurtlesimNode();

      const t = createMockTerminal();
      await handleRos2Param(['set', '/turtlesim', 'background_r', '150'], t);

      const node = ProcessManager._state.processes['/turtlesim'].node;
      expect(node.setParameter).toHaveBeenCalledWith('background_r', 150);
      expect(t.lines[0]).toContain('Set parameter');
    });
  });

  describe('ros2 param dump', () => {
    it('dumps all parameters in YAML-like format', async () => {
      setupTurtlesimNode();

      const t = createMockTerminal();
      await handleRos2Param(['dump', '/turtlesim'], t);

      expect(t.lines[0]).toBe('/turtlesim:');
      expect(t.lines[1]).toBe('  ros__parameters:');
      // Parameters at 4-space indent
      expect(t.lines).toContain('    background_b: 255');
      expect(t.lines).toContain('    background_g: 86');
      expect(t.lines).toContain('    background_r: 69');
    });
  });
});

// =====================================================================
// Tutorial 5: Understanding Actions
// =====================================================================
describe('ros2 action (Tutorial: Understanding Actions)', () => {
  describe('ros2 action list', () => {
    it('lists actions without types', async () => {
      SimDDS._state.actions = [
        { name: '/turtle1/rotate_absolute', type: 'turtlesim/action/RotateAbsolute' },
      ];

      const t = createMockTerminal();
      await handleRos2Action(['list'], t);

      expect(t.lines).toContain('/turtle1/rotate_absolute');
      expect(t.lines.join('')).not.toContain('[');
    });

    it('lists actions with types when -t flag is passed', async () => {
      SimDDS._state.actions = [
        { name: '/turtle1/rotate_absolute', type: 'turtlesim/action/RotateAbsolute' },
      ];

      const t = createMockTerminal();
      await handleRos2Action(['list', '-t'], t);

      expect(t.lines).toContain('/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]');
    });
  });

  describe('ros2 action type', () => {
    it('shows the type of an action', async () => {
      SimDDS._state.actionInfo['/turtle1/rotate_absolute'] = {
        name: '/turtle1/rotate_absolute',
        type: 'turtlesim/action/RotateAbsolute',
        node: '/turtlesim',
      };

      const t = createMockTerminal();
      await handleRos2Action(['type', '/turtle1/rotate_absolute'], t);

      expect(t.lines).toContain('turtlesim/action/RotateAbsolute');
    });
  });

  describe('ros2 action info', () => {
    it('shows action info matching real ros2 CLI format', async () => {
      SimDDS._state.actionInfo['/turtle1/rotate_absolute'] = {
        name: '/turtle1/rotate_absolute',
        type: 'turtlesim/action/RotateAbsolute',
        node: '/turtlesim',
      };

      const t = createMockTerminal();
      await handleRos2Action(['info', '/turtle1/rotate_absolute'], t);

      // Real ros2 CLI format
      expect(t.lines[0]).toBe('Action: /turtle1/rotate_absolute');
      expect(t.lines[1]).toBe('Action clients: 0');
      expect(t.lines[2]).toBe('Action servers: 1');
      expect(t.lines[3]).toBe('    /turtlesim');
    });

    it('shows error for unknown action', async () => {
      const t = createMockTerminal();
      await handleRos2Action(['info', '/nonexistent'], t);

      expect(t.lines[0]).toContain('not found');
    });
  });

  describe('ros2 action send_goal', () => {
    it('sends a goal and shows the result', async () => {
      SimDDS._state.actionInfo['/turtle1/rotate_absolute'] = {
        name: '/turtle1/rotate_absolute',
        type: 'turtlesim/action/RotateAbsolute',
        node: '/turtlesim',
      };
      SimDDS._state.sendActionGoalResult = { delta: 0.5 };

      const t = createMockTerminal();
      await handleRos2Action([
        'send_goal', '/turtle1/rotate_absolute',
        'turtlesim/action/RotateAbsolute',
        '{"theta": 1.57}',
      ], t);

      expect(SimDDS.sendActionGoal).toHaveBeenCalled();
      expect(t.lines.join('\n')).toContain('Goal accepted');
      expect(t.lines.join('\n')).toContain('Result:');
      expect(t.lines.join('\n')).toContain('SUCCEEDED');
    });

    it('shows feedback when --feedback flag is passed', async () => {
      SimDDS._state.actionInfo['/turtle1/rotate_absolute'] = {
        name: '/turtle1/rotate_absolute',
        type: 'turtlesim/action/RotateAbsolute',
        node: '/turtlesim',
      };
      SimDDS._state.sendActionGoalResult = { delta: 0.5 };

      const t = createMockTerminal();
      await handleRos2Action([
        'send_goal', '/turtle1/rotate_absolute',
        'turtlesim/action/RotateAbsolute',
        '{"theta": -1.57}',
        '--feedback',
      ], t);

      // The mock sendActionGoal calls feedbackCallback
      expect(t.lines.join('\n')).toContain('Feedback:');
    });

    it('shows error for unknown action type', async () => {
      const t = createMockTerminal();
      await handleRos2Action([
        'send_goal', '/turtle1/rotate_absolute',
        'fake/action/Nope', '{}',
      ], t);

      expect(t.lines[0]).toContain('Unknown action type');
    });
  });
});

// =====================================================================
// ros2 interface (used across multiple tutorials)
// =====================================================================
describe('ros2 interface (used across tutorials)', () => {
  describe('ros2 interface show', () => {
    it('shows message definition for geometry_msgs/msg/Twist', async () => {
      const t = createMockTerminal();
      await handleRos2Interface(['show', 'geometry_msgs/msg/Twist'], t);

      expect(t.lines[0]).toContain('geometry_msgs/Vector3 linear');
    });

    it('shows service definition with --- separator', async () => {
      const t = createMockTerminal();
      await handleRos2Interface(['show', 'std_srvs/srv/Empty'], t);

      expect(t.lines).toContain('---');
    });

    it('shows action definition for turtlesim/action/RotateAbsolute', async () => {
      const t = createMockTerminal();
      await handleRos2Interface(['show', 'turtlesim/action/RotateAbsolute'], t);

      // Action definitions have two --- separators
      const output = t.lines[0]; // getDefinition returns one string
      expect(output).toContain('theta');
      expect(output).toContain('---');
    });

    it('shows error for unknown interface', async () => {
      const t = createMockTerminal();
      await handleRos2Interface(['show', 'fake/msg/Missing'], t);

      expect(t.lines[0]).toContain('not found');
    });
  });

  describe('ros2 interface list', () => {
    it('lists messages, services, and actions with 4-space indent', async () => {
      const t = createMockTerminal();
      await handleRos2Interface(['list'], t);

      expect(t.lines).toContain('Messages:');
      expect(t.lines).toContain('    geometry_msgs/msg/Twist');
      expect(t.lines).toContain('    std_msgs/msg/String');
      expect(t.lines).toContain('Services:');
      expect(t.lines).toContain('    std_srvs/srv/Empty');
      expect(t.lines).toContain('    turtlesim/srv/Spawn');
      expect(t.lines).toContain('Actions:');
      expect(t.lines).toContain('    turtlesim/action/RotateAbsolute');
    });

    it('filters to only messages with -m flag', async () => {
      const t = createMockTerminal();
      await handleRos2Interface(['list', '-m'], t);

      expect(t.lines).toContain('Messages:');
      expect(t.lines).not.toContain('Services:');
      expect(t.lines).not.toContain('Actions:');
    });

    it('filters to only services with -s flag', async () => {
      const t = createMockTerminal();
      await handleRos2Interface(['list', '-s'], t);

      expect(t.lines).not.toContain('Messages:');
      expect(t.lines).toContain('Services:');
      expect(t.lines).not.toContain('Actions:');
    });
  });
});

// =====================================================================
// Help / error handling across all commands
// =====================================================================
describe('help and error handling', () => {
  it('ros2 node shows help when no subcommand given', async () => {
    const t = createMockTerminal();
    await handleRos2Node([], t);
    expect(t.lines.join('\n')).toContain('usage:');
  });

  it('ros2 topic shows help when no subcommand given', async () => {
    const t = createMockTerminal();
    await handleRos2Topic([], t);
    expect(t.lines.join('\n')).toContain('usage:');
  });

  it('ros2 service shows help when no subcommand given', async () => {
    const t = createMockTerminal();
    await handleRos2Service([], t);
    expect(t.lines.join('\n')).toContain('usage:');
  });

  it('ros2 param shows help when no subcommand given', async () => {
    const t = createMockTerminal();
    await handleRos2Param([], t);
    expect(t.lines.join('\n')).toContain('usage:');
  });

  it('ros2 action shows help when no subcommand given', async () => {
    const t = createMockTerminal();
    await handleRos2Action([], t);
    expect(t.lines.join('\n')).toContain('usage:');
  });

  it('ros2 interface shows help when no subcommand given', async () => {
    const t = createMockTerminal();
    await handleRos2Interface([], t);
    expect(t.lines.join('\n')).toContain('usage:');
  });

  it('ros2 node shows error for unknown subcommand', async () => {
    const t = createMockTerminal();
    await handleRos2Node(['bogus'], t);
    expect(t.lines[0]).toContain('Unknown subcommand');
  });

  it('ros2 topic shows error for unknown subcommand', async () => {
    const t = createMockTerminal();
    await handleRos2Topic(['bogus'], t);
    expect(t.lines[0]).toContain('Unknown subcommand');
  });

  it('ros2 bag shows help when no subcommand given', async () => {
    const t = createMockTerminal();
    await handleRos2Bag([], t);
    expect(t.lines.join('\n')).toContain('usage:');
  });
});

// =====================================================================
// SLAM Tutorial: CLI commands a student would run
// =====================================================================
describe('SLAM Tutorial CLI commands', () => {
  // Helper: set up the full SLAM scenario (turtlesim + slam_node running)
  const setupSlamScenario = () => {
    SimDDS._state.nodes = ['/turtlesim_node', '/slam_node', '/teleop_twist_keyboard'];

    // Topics active during SLAM
    SimDDS._state.topics = [
      { name: '/scan', type: 'sensor_msgs/msg/LaserScan' },
      { name: '/turtle1/pose', type: 'turtlesim/msg/Pose' },
      { name: '/turtle1/cmd_vel', type: 'geometry_msgs/msg/Twist' },
      { name: '/turtle1/color_sensor', type: 'turtlesim/msg/Color' },
      { name: '/map', type: 'nav_msgs/msg/OccupancyGrid' },
      { name: '/rosout', type: 'rcl_interfaces/msg/Log' },
    ];

    SimDDS._state.topicInfo['/scan'] = {
      name: '/scan',
      type: 'sensor_msgs/msg/LaserScan',
      publishers: [{ nodeId: '/turtlesim_node' }],
      subscribers: [{ nodeId: '/slam_node' }],
    };
    SimDDS._state.topicInfo['/map'] = {
      name: '/map',
      type: 'nav_msgs/msg/OccupancyGrid',
      publishers: [{ nodeId: '/slam_node' }],
      subscribers: [],
    };
    SimDDS._state.topicInfo['/turtle1/pose'] = {
      name: '/turtle1/pose',
      type: 'turtlesim/msg/Pose',
      publishers: [{ nodeId: '/turtlesim_node' }],
      subscribers: [{ nodeId: '/slam_node' }],
    };
    SimDDS._state.topicInfo['/turtle1/cmd_vel'] = {
      name: '/turtle1/cmd_vel',
      type: 'geometry_msgs/msg/Twist',
      publishers: [{ nodeId: '/teleop_twist_keyboard' }],
      subscribers: [{ nodeId: '/turtlesim_node' }],
    };

    // Services
    SimDDS._state.services = [
      { name: '/clear', type: 'std_srvs/srv/Empty', node: '/turtlesim_node' },
      { name: '/spawn', type: 'turtlesim/srv/Spawn', node: '/turtlesim_node' },
    ];
    SimDDS._state.serviceInfo['/clear'] = {
      name: '/clear', type: 'std_srvs/srv/Empty', node: '/turtlesim_node',
    };

    // Actions
    SimDDS._state.actions = [
      { name: '/turtle1/rotate_absolute', type: 'turtlesim/action/RotateAbsolute' },
    ];
    SimDDS._state.actionInfo['/turtle1/rotate_absolute'] = {
      name: '/turtle1/rotate_absolute',
      type: 'turtlesim/action/RotateAbsolute',
      node: '/turtlesim_node',
    };

    // Node info for slam_node
    SimDDS._state.nodeInfo['/slam_node'] = { name: '/slam_node' };
    ProcessManager._state.processes['/slam_node'] = {
      node: {
        subscriptions: [
          { topic: '/turtle1/pose', msgType: 'turtlesim/msg/Pose' },
          { topic: '/scan', msgType: 'sensor_msgs/msg/LaserScan' },
        ],
        publishers: [
          { topic: '/map', msgType: 'nav_msgs/msg/OccupancyGrid' },
        ],
        services: [],
        actionServers: [],
        getParameterNames: () => [
          'map_resolution', 'map_width', 'map_height',
          'origin_x', 'origin_y', 'hit_prob', 'miss_prob',
        ],
        getParameter: (name) => {
          const params = {
            map_resolution: 0.1, map_width: 110, map_height: 110,
            origin_x: 0.0, origin_y: 0.0, hit_prob: 0.9, miss_prob: 0.3,
          };
          return params[name];
        },
        setParameter: vi.fn(() => true),
      },
    };

    // Node info for turtlesim
    SimDDS._state.nodeInfo['/turtlesim_node'] = { name: '/turtlesim_node' };
    ProcessManager._state.processes['/turtlesim_node'] = {
      node: {
        subscriptions: [
          { topic: '/turtle1/cmd_vel', msgType: 'geometry_msgs/msg/Twist' },
        ],
        publishers: [
          { topic: '/turtle1/pose', msgType: 'turtlesim/msg/Pose' },
          { topic: '/turtle1/color_sensor', msgType: 'turtlesim/msg/Color' },
          { topic: '/scan', msgType: 'sensor_msgs/msg/LaserScan' },
        ],
        services: [
          { name: '/clear', type: 'std_srvs/srv/Empty' },
          { name: '/spawn', type: 'turtlesim/srv/Spawn' },
        ],
        actionServers: [
          { name: '/turtle1/rotate_absolute', type: 'turtlesim/action/RotateAbsolute' },
        ],
        getParameterNames: () => ['background_r', 'background_g', 'background_b'],
        getParameter: (name) => {
          const params = { background_r: 69, background_g: 86, background_b: 255 };
          return params[name];
        },
        setParameter: vi.fn(() => true),
      },
    };
  };

  describe('Step 6: Inspect the system', () => {
    it('ros2 node list shows all three SLAM nodes', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Node(['list'], t);

      expect(t.lines).toContain('/slam_node');
      expect(t.lines).toContain('/turtlesim_node');
      expect(t.lines).toContain('/teleop_twist_keyboard');
    });

    it('ros2 node info /slam_node shows subscriptions and publishers', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Node(['info', '/slam_node'], t);

      expect(t.lines[0]).toBe('/slam_node');
      expect(t.lines).toContain('  Subscribers:');
      expect(t.lines).toContain('    /turtle1/pose: turtlesim/msg/Pose');
      expect(t.lines).toContain('    /scan: sensor_msgs/msg/LaserScan');
      expect(t.lines).toContain('  Publishers:');
      expect(t.lines).toContain('    /map: nav_msgs/msg/OccupancyGrid');
    });

    it('ros2 topic list -t shows SLAM-related topics with types', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Topic(['list', '-t'], t);

      expect(t.lines).toContain('/scan [sensor_msgs/msg/LaserScan]');
      expect(t.lines).toContain('/map [nav_msgs/msg/OccupancyGrid]');
      expect(t.lines).toContain('/turtle1/pose [turtlesim/msg/Pose]');
      expect(t.lines).toContain('/turtle1/cmd_vel [geometry_msgs/msg/Twist]');
    });
  });

  describe('Step 7: Monitor live data', () => {
    it('ros2 topic echo /scan subscribes to lidar data', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Topic(['echo', '/scan'], t);

      expect(SimDDS.subscribe).toHaveBeenCalledWith(
        '/scan', 'sensor_msgs/msg/LaserScan', expect.any(Function),
      );
    });

    it('ros2 topic echo /map subscribes to occupancy grid', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Topic(['echo', '/map'], t);

      expect(SimDDS.subscribe).toHaveBeenCalledWith(
        '/map', 'nav_msgs/msg/OccupancyGrid', expect.any(Function),
      );
    });

    it('ros2 topic info /scan shows publisher and subscriber counts', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Topic(['info', '/scan'], t);

      expect(t.lines).toContain('Type: sensor_msgs/msg/LaserScan');
      expect(t.lines).toContain('Publisher count: 1');
      expect(t.lines).toContain('Subscription count: 1');
    });

    it('ros2 topic info /map shows publisher count', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Topic(['info', '/map'], t);

      expect(t.lines).toContain('Type: nav_msgs/msg/OccupancyGrid');
      expect(t.lines).toContain('Publisher count: 1');
      expect(t.lines).toContain('Subscription count: 0');
    });

    it('ros2 topic hz /scan subscribes to measure rate', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Topic(['hz', '/scan'], t);

      expect(SimDDS.subscribe).toHaveBeenCalled();
      expect(t.finishCommand).not.toHaveBeenCalled();
    });

    it('ros2 topic bw /scan subscribes to measure bandwidth', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Topic(['bw', '/scan'], t);

      expect(SimDDS.subscribe).toHaveBeenCalled();
      expect(t.finishCommand).not.toHaveBeenCalled();
    });
  });

  describe('Step 8: Tune SLAM parameters', () => {
    it('ros2 param list /slam_node shows all SLAM parameters', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Param(['list', '/slam_node'], t);

      expect(t.lines[0]).toBe('/slam_node:');
      expect(t.lines).toContain('  hit_prob');
      expect(t.lines).toContain('  map_resolution');
      expect(t.lines).toContain('  miss_prob');
      expect(t.lines).toContain('  map_width');
      expect(t.lines).toContain('  map_height');
    });

    it('ros2 param get /slam_node map_resolution returns 0.1', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Param(['get', '/slam_node', 'map_resolution'], t);

      expect(t.lines[0]).toBe('Double value is: 0.100000');
    });

    it('ros2 param get /slam_node hit_prob returns 0.9', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Param(['get', '/slam_node', 'hit_prob'], t);

      expect(t.lines[0]).toBe('Double value is: 0.900000');
    });

    it('ros2 param set /slam_node hit_prob 0.95 updates the parameter', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Param(['set', '/slam_node', 'hit_prob', '0.95'], t);

      const node = ProcessManager._state.processes['/slam_node'].node;
      expect(node.setParameter).toHaveBeenCalledWith('hit_prob', 0.95);
      expect(t.lines[0]).toContain('Set parameter');
    });

    it('ros2 param set /slam_node miss_prob 0.1 updates the parameter', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Param(['set', '/slam_node', 'miss_prob', '0.1'], t);

      const node = ProcessManager._state.processes['/slam_node'].node;
      expect(node.setParameter).toHaveBeenCalledWith('miss_prob', 0.1);
    });

    it('ros2 param dump /slam_node shows all parameter values', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Param(['dump', '/slam_node'], t);

      expect(t.lines[0]).toBe('/slam_node:');
      expect(t.lines[1]).toBe('  ros__parameters:');
      expect(t.lines).toContain('    hit_prob: 0.900000');
      expect(t.lines).toContain('    map_resolution: 0.100000');
      expect(t.lines).toContain('    map_width: 110');
    });
  });

  describe('Step 9: Record and replay', () => {
    it('ros2 bag record starts recording specified topics', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Bag(['record', '-o', 'my_slam_run', '/scan', '/turtle1/pose', '/map'], t);

      expect(t.lines.join('\n')).toContain('Recording');
      expect(t.lines.join('\n')).toContain('my_slam_run');
      expect(t.lines).toContain('  /scan');
      expect(t.lines).toContain('  /turtle1/pose');
      expect(t.lines).toContain('  /map');
      expect(t.addBagRecorder).toHaveBeenCalled();
    });

    it('ros2 bag record -a records all active topics', async () => {
      setupSlamScenario();
      const t = createMockTerminal();
      await handleRos2Bag(['record', '-a'], t);

      expect(t.lines.join('\n')).toContain('Recording');
      // Should list all 6 topics from the scenario
      expect(t.lines).toContain('  /scan');
      expect(t.lines).toContain('  /map');
    });

    it('ros2 bag info shows bag metadata', async () => {
      // Pre-store a bag
      BagStorage._bags['my_slam_run'] = {
        name: 'my_slam_run',
        getInfo: () => ({
          name: 'my_slam_run',
          duration: 10.5,
          messageCount: 42,
          topics: ['/scan', '/turtle1/pose', '/map'],
          topicCounts: { '/scan': 20, '/turtle1/pose': 15, '/map': 7 },
        }),
      };

      const t = createMockTerminal();
      await handleRos2Bag(['info', 'my_slam_run'], t);

      expect(t.lines).toContain('Bag: my_slam_run');
      expect(t.lines).toContain('Duration: 10.50s');
      expect(t.lines).toContain('Messages: 42');
      expect(t.lines).toContain('Topics:');
      expect(t.lines).toContain('  /scan: 20 messages');
      expect(t.lines).toContain('  /turtle1/pose: 15 messages');
      expect(t.lines).toContain('  /map: 7 messages');
    });

    it('ros2 bag play replays a recorded bag', async () => {
      BagStorage._bags['my_slam_run'] = {
        name: 'my_slam_run',
        getInfo: () => ({
          name: 'my_slam_run',
          duration: 10.5,
          messageCount: 42,
          topics: ['/scan'],
          topicCounts: { '/scan': 42 },
        }),
      };

      const t = createMockTerminal();
      await handleRos2Bag(['play', 'my_slam_run'], t);

      expect(t.lines.join('\n')).toContain('Playing');
      expect(t.lines.join('\n')).toContain('my_slam_run');
      expect(t.lines.join('\n')).toContain('Playback complete');
    });

    it('ros2 bag play with --rate flag sets playback speed', async () => {
      BagStorage._bags['my_slam_run'] = {
        name: 'my_slam_run',
        getInfo: () => ({
          name: 'my_slam_run',
          duration: 10.5,
          messageCount: 42,
          topics: ['/scan'],
          topicCounts: { '/scan': 42 },
        }),
      };

      const t = createMockTerminal();
      await handleRos2Bag(['play', 'my_slam_run', '--rate', '0.5'], t);

      expect(t.lines.join('\n')).toContain('Rate: 0.5x');
    });

    it('ros2 bag info shows error for nonexistent bag', async () => {
      const t = createMockTerminal();
      await handleRos2Bag(['info', 'nonexistent'], t);

      expect(t.lines[0]).toContain('not found');
    });

    it('ros2 bag play shows error for nonexistent bag', async () => {
      const t = createMockTerminal();
      await handleRos2Bag(['play', 'nonexistent'], t);

      expect(t.lines[0]).toContain('not found');
    });
  });

  describe('Services and actions during SLAM', () => {
    it('ros2 service call /clear clears the drawing', async () => {
      setupSlamScenario();
      SimDDS._state.callServiceResult = {};
      const t = createMockTerminal();
      await handleRos2Service(['call', '/clear', 'std_srvs/srv/Empty'], t);

      expect(SimDDS.callService).toHaveBeenCalledWith(
        '/clear', 'std_srvs/srv/Empty', expect.any(Object),
      );
    });

    it('ros2 action send_goal rotates the turtle', async () => {
      setupSlamScenario();
      SimDDS._state.sendActionGoalResult = { delta: 0.0 };
      const t = createMockTerminal();
      await handleRos2Action([
        'send_goal', '/turtle1/rotate_absolute',
        'turtlesim/action/RotateAbsolute',
        '{"theta": 1.57}',
      ], t);

      expect(SimDDS.sendActionGoal).toHaveBeenCalled();
      expect(t.lines.join('\n')).toContain('SUCCEEDED');
    });
  });

  describe('Interface inspection during SLAM', () => {
    it('ros2 interface show sensor_msgs/msg/LaserScan shows lidar definition', async () => {
      // The mock getDefinition doesn't have LaserScan, so this tests the error path
      // which is still useful - students would see "not found" for unregistered types
      const { getDefinition } = await import('../msgs/index.js');
      getDefinition.mockReturnValueOnce(
        'float32 angle_min\nfloat32 angle_max\nfloat32 angle_increment\n' +
        'float32 time_increment\nfloat32 scan_time\nfloat32 range_min\n' +
        'float32 range_max\nfloat32[] ranges\nfloat32[] intensities'
      );

      const t = createMockTerminal();
      await handleRos2Interface(['show', 'sensor_msgs/msg/LaserScan'], t);

      expect(t.lines[0]).toContain('angle_min');
    });

    it('ros2 interface show nav_msgs/msg/OccupancyGrid shows map definition', async () => {
      const { getDefinition } = await import('../msgs/index.js');
      getDefinition.mockReturnValueOnce(
        'std_msgs/Header header\nnav_msgs/MapMetaData info\nint8[] data'
      );

      const t = createMockTerminal();
      await handleRos2Interface(['show', 'nav_msgs/msg/OccupancyGrid'], t);

      expect(t.lines[0]).toContain('header');
    });
  });
});
