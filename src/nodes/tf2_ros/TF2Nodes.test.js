import { describe, it, expect, beforeEach, vi, afterEach } from 'vitest';

// Mock TFBuffer
vi.mock('../../core/TFBuffer.js', () => {
  const state = {
    transform: null,
    tree: {},
    frames: new Set(),
  };
  return {
    TFBuffer: {
      _state: state,
      lookupTransform: vi.fn((target, source) => state.transform),
      getFrameTree: vi.fn(() => state.tree),
      _getAllFrames: vi.fn(() => state.frames),
      setTransform: vi.fn(),
    },
  };
});

// Mock SimDDS
vi.mock('../../core/SimDDS.js', () => ({
  SimDDS: {
    registerNode: vi.fn(),
    unregisterNode: vi.fn(),
    createPublisher: vi.fn(() => ({ id: 'pub1', publish: vi.fn(), destroy: vi.fn() })),
    createSubscription: vi.fn((nodeId, topic, type, cb) => {
      return { id: 'sub1', topic, callback: cb, destroy: vi.fn() };
    }),
    createService: vi.fn(() => ({ id: 'srv1', destroy: vi.fn() })),
    createActionServer: vi.fn(() => ({ id: 'act1', destroy: vi.fn() })),
  },
}));

// Mock LogManager
vi.mock('../../core/LogManager.js', () => ({
  LogManager: {
    getLogger: () => {
      const logs = [];
      return {
        _logs: logs,
        debug: (msg) => logs.push({ level: 'debug', msg }),
        info: (msg) => logs.push({ level: 'info', msg }),
        warn: (msg) => logs.push({ level: 'warn', msg }),
        error: (msg) => logs.push({ level: 'error', msg }),
        fatal: (msg) => logs.push({ level: 'fatal', msg }),
      };
    },
  },
}));

vi.mock('../../core/ServiceContainer.js', () => ({
  ServiceContainer: { get: () => null, register: vi.fn() },
}));

vi.mock('../../core/ProcessManager.js', () => ({
  ProcessManager: {
    getProcessByNodeName: vi.fn(() => null),
    kill: vi.fn(),
  },
}));

import { TFBuffer } from '../../core/TFBuffer.js';
import { SimDDS } from '../../core/SimDDS.js';
import { TF2EchoNode } from './TF2EchoNode.js';
import { TF2MonitorNode } from './TF2MonitorNode.js';
import { ViewFramesNode } from './ViewFramesNode.js';

describe('TF2EchoNode', () => {
  beforeEach(() => {
    vi.useFakeTimers();
    TFBuffer._state.transform = null;
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it('parses source and target frames from args', () => {
    const node = new TF2EchoNode('tf2_echo', { args: ['world', 'base_link'] });
    expect(node.sourceFrame).toBe('world');
    expect(node.targetFrame).toBe('base_link');
  });

  it('uses defaults when no args provided', () => {
    const node = new TF2EchoNode('tf2_echo', {});
    expect(node.sourceFrame).toBe('world');
    expect(node.targetFrame).toBe('base_link');
  });

  it('logs transform when available', () => {
    TFBuffer._state.transform = {
      translation: { x: 1.0, y: 2.0, z: 0.0 },
      rotation: { x: 0, y: 0, z: 0, w: 1 },
    };

    const node = new TF2EchoNode('tf2_echo', { args: ['world', 'base_link'] });
    node.running = true;
    node.onInit();

    vi.advanceTimersByTime(1000);

    expect(TFBuffer.lookupTransform).toHaveBeenCalledWith('base_link', 'world');
    const logs = node.logger._logs;
    const translationLog = logs.find(l => l.msg.includes('Translation'));
    expect(translationLog).toBeDefined();
    expect(translationLog.msg).toContain('1.000');
    expect(translationLog.msg).toContain('2.000');
  });

  it('logs warning when transform not available', () => {
    TFBuffer._state.transform = null;

    const node = new TF2EchoNode('tf2_echo', { args: ['world', 'missing'] });
    node.running = true;
    node.onInit();

    vi.advanceTimersByTime(1000);

    const logs = node.logger._logs;
    const warnLog = logs.find(l => l.level === 'warn');
    expect(warnLog).toBeDefined();
    expect(warnLog.msg).toContain('not found');
  });
});

describe('ViewFramesNode', () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it('logs frame tree when frames exist', () => {
    TFBuffer._state.tree = {
      world: { parents: [], children: ['base_link'] },
      base_link: { parents: ['world'], children: ['laser'] },
      laser: { parents: ['base_link'], children: [] },
    };

    const node = new ViewFramesNode('view_frames', {});
    node.running = true;
    node.onInit();

    const logs = node.logger._logs;
    const frameLog = logs.find(l => l.msg.includes('Frames (3)'));
    expect(frameLog).toBeDefined();

    // Should show tree structure
    const worldLog = logs.find(l => l.msg.includes('world'));
    expect(worldLog).toBeDefined();
    const baseLinkLog = logs.find(l => l.msg.includes('base_link'));
    expect(baseLinkLog).toBeDefined();
    const laserLog = logs.find(l => l.msg.includes('laser'));
    expect(laserLog).toBeDefined();
  });

  it('logs message when no frames exist', () => {
    TFBuffer._state.tree = {};

    const node = new ViewFramesNode('view_frames', {});
    node.running = true;
    node.onInit();

    const logs = node.logger._logs;
    const noDataLog = logs.find(l => l.msg.includes('No transform data'));
    expect(noDataLog).toBeDefined();
  });
});

describe('TF2MonitorNode', () => {
  beforeEach(() => {
    vi.useFakeTimers();
    TFBuffer._state.tree = {};
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it('creates subscriptions to /tf and /tf_static', () => {
    SimDDS.createSubscription.mockClear();

    const node = new TF2MonitorNode('tf2_monitor', {});
    node.running = true;
    node.onInit();

    const calls = SimDDS.createSubscription.mock.calls;
    const tfCall = calls.find(c => c[1] === '/tf');
    const tfStaticCall = calls.find(c => c[1] === '/tf_static');
    expect(tfCall).toBeDefined();
    expect(tfStaticCall).toBeDefined();
  });

  it('displays status periodically', () => {
    TFBuffer._state.tree = { world: { parents: [], children: [] } };

    const node = new TF2MonitorNode('tf2_monitor', {});
    node.running = true;
    node.onInit();

    vi.advanceTimersByTime(1000);

    const logs = node.logger._logs;
    const framesLog = logs.find(l => l.msg.includes('Frames:'));
    expect(framesLog).toBeDefined();
  });
});
