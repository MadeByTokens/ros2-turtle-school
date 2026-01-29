import { describe, it, expect, beforeEach, vi, afterEach } from 'vitest';
import { Node } from './Node.js';

describe('Node', () => {
  let mockSimDDS;
  let mockLogManager;
  let mockLogger;

  beforeEach(() => {
    // Create mock logger
    mockLogger = {
      debug: vi.fn(),
      info: vi.fn(),
      warn: vi.fn(),
      error: vi.fn(),
      fatal: vi.fn()
    };

    // Create mock LogManager
    mockLogManager = {
      getLogger: vi.fn().mockReturnValue(mockLogger)
    };

    // Create mock SimDDS
    mockSimDDS = {
      registerNode: vi.fn(),
      unregisterNode: vi.fn(),
      createPublisher: vi.fn().mockReturnValue({ id: 'pub_1' }),
      createSubscription: vi.fn().mockReturnValue({ id: 'sub_1' }),
      createService: vi.fn().mockReturnValue({ id: 'srv_1' }),
      createActionServer: vi.fn().mockReturnValue({ id: 'action_1' })
    };
  });

  afterEach(() => {
    vi.clearAllTimers();
    vi.useRealTimers();
  });

  describe('constructor', () => {
    it('creates node with correct name', () => {
      const node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
      expect(node.name).toBe('test_node');
      expect(node.fullName).toBe('/test_node');
    });

    it('handles namespace', () => {
      const node = new Node('test_node', {
        simDDS: mockSimDDS,
        logManager: mockLogManager,
        namespace: '/my_ns'
      });
      expect(node.fullName).toBe('/my_ns/test_node');
    });

    it('registers with SimDDS', () => {
      new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
      expect(mockSimDDS.registerNode).toHaveBeenCalledWith('/test_node', {
        name: 'test_node',
        namespace: ''
      });
    });

    it('initializes logger', () => {
      new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
      expect(mockLogManager.getLogger).toHaveBeenCalledWith('/test_node');
    });

    it('initializes parameters from options', () => {
      const node = new Node('test_node', {
        simDDS: mockSimDDS,
        logManager: mockLogManager,
        parameters: { speed: 1.0, name: 'turtle1' }
      });

      expect(node.getParameter('speed')).toBe(1.0);
      expect(node.getParameter('name')).toBe('turtle1');
    });
  });

  describe('declareParameter / getParameter / setParameter', () => {
    let node;

    beforeEach(() => {
      node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
    });

    it('declares a parameter with default value', () => {
      const value = node.declareParameter('speed', 2.0);
      expect(value).toBe(2.0);
      expect(node.getParameter('speed')).toBe(2.0);
    });

    it('does not overwrite existing parameter on re-declare', () => {
      node.declareParameter('speed', 2.0);
      node.setParameter('speed', 5.0);

      const value = node.declareParameter('speed', 1.0);
      expect(value).toBe(5.0); // Should keep existing value
    });

    it('returns undefined for non-existent parameter', () => {
      expect(node.getParameter('nonexistent')).toBeUndefined();
    });

    it('sets parameter value', () => {
      node.declareParameter('speed', 1.0);
      const result = node.setParameter('speed', 3.0);

      expect(result).toBe(true);
      expect(node.getParameter('speed')).toBe(3.0);
    });

    it('returns false when setting non-existent parameter', () => {
      const result = node.setParameter('nonexistent', 1.0);
      expect(result).toBe(false);
    });

    it('calls onParameterChange when parameter is set', () => {
      const spy = vi.spyOn(node, 'onParameterChange');
      node.declareParameter('speed', 1.0);
      node.setParameter('speed', 2.0);

      expect(spy).toHaveBeenCalledWith('speed', 2.0);
    });
  });

  describe('getParameterNames', () => {
    it('returns all parameter names', () => {
      const node = new Node('test_node', {
        simDDS: mockSimDDS,
        logManager: mockLogManager,
        parameters: { a: 1, b: 2, c: 3 }
      });

      const names = node.getParameterNames();
      expect(names).toContain('a');
      expect(names).toContain('b');
      expect(names).toContain('c');
      expect(names).toHaveLength(3);
    });

    it('returns empty array when no parameters', () => {
      const node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
      expect(node.getParameterNames()).toEqual([]);
    });
  });

  describe('createTimer / cancelTimer', () => {
    let node;

    beforeEach(() => {
      vi.useFakeTimers();
      node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
      node.start();
    });

    it('creates a timer that fires periodically', () => {
      const callback = vi.fn();
      node.createTimer(100, callback);

      vi.advanceTimersByTime(100);
      expect(callback).toHaveBeenCalledTimes(1);

      vi.advanceTimersByTime(100);
      expect(callback).toHaveBeenCalledTimes(2);
    });

    it('timer only fires when node is running', () => {
      const callback = vi.fn();
      node.createTimer(100, callback);

      vi.advanceTimersByTime(100);
      expect(callback).toHaveBeenCalledTimes(1);

      node.running = false;
      vi.advanceTimersByTime(100);
      expect(callback).toHaveBeenCalledTimes(1); // Not called again
    });

    it('cancels a timer', () => {
      const callback = vi.fn();
      const timerId = node.createTimer(100, callback);

      vi.advanceTimersByTime(100);
      expect(callback).toHaveBeenCalledTimes(1);

      node.cancelTimer(timerId);

      vi.advanceTimersByTime(100);
      expect(callback).toHaveBeenCalledTimes(1); // Still 1
    });

    it('removes timer from list on cancel', () => {
      const timerId = node.createTimer(100, () => {});
      expect(node.timers).toContain(timerId);

      node.cancelTimer(timerId);
      expect(node.timers).not.toContain(timerId);
    });
  });

  describe('start / destroy lifecycle', () => {
    let node;

    beforeEach(() => {
      node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
    });

    it('start sets running to true and calls onInit', () => {
      const onInitSpy = vi.spyOn(node, 'onInit');

      node.start();

      expect(node.running).toBe(true);
      expect(onInitSpy).toHaveBeenCalled();
      expect(mockLogger.info).toHaveBeenCalledWith('Node started');
    });

    it('destroy sets running to false and calls onShutdown', () => {
      const onShutdownSpy = vi.spyOn(node, 'onShutdown');
      node.start();

      node.destroy();

      expect(node.running).toBe(false);
      expect(onShutdownSpy).toHaveBeenCalled();
      expect(mockLogger.info).toHaveBeenCalledWith('Node destroyed');
    });

    it('destroy cancels all timers', () => {
      vi.useFakeTimers();
      node.start();

      const callback = vi.fn();
      node.createTimer(100, callback);
      node.createTimer(100, callback);

      node.destroy();

      expect(node.timers).toHaveLength(0);

      vi.advanceTimersByTime(200);
      expect(callback).not.toHaveBeenCalled();
    });

    it('destroy clears publishers, subscriptions, services, actions', () => {
      node.start();
      node.createPublisher('/topic', 'type');
      node.createSubscription('/topic', 'type', () => {});
      node.createService('/srv', 'type', () => {});
      node.createActionServer('/action', 'type', { executeCallback: () => {} });

      node.destroy();

      expect(node.publishers).toHaveLength(0);
      expect(node.subscriptions).toHaveLength(0);
      expect(node.services).toHaveLength(0);
      expect(node.actionServers).toHaveLength(0);
    });

    it('destroy unregisters from SimDDS', () => {
      node.start();
      node.destroy();

      expect(mockSimDDS.unregisterNode).toHaveBeenCalledWith('/test_node');
    });
  });

  describe('now', () => {
    it('returns timestamp in ROS format', () => {
      const node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
      const now = node.now();

      expect(now).toHaveProperty('sec');
      expect(now).toHaveProperty('nanosec');
      expect(typeof now.sec).toBe('number');
      expect(typeof now.nanosec).toBe('number');
    });

    it('returns reasonable timestamp values', () => {
      const node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
      const before = Math.floor(Date.now() / 1000);
      const now = node.now();
      const after = Math.floor(Date.now() / 1000);

      expect(now.sec).toBeGreaterThanOrEqual(before);
      expect(now.sec).toBeLessThanOrEqual(after);
      expect(now.nanosec).toBeGreaterThanOrEqual(0);
      expect(now.nanosec).toBeLessThan(1000000000);
    });
  });

  describe('logging methods', () => {
    let node;

    beforeEach(() => {
      node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
    });

    it('logDebug delegates to logger', () => {
      node.logDebug('debug message');
      expect(mockLogger.debug).toHaveBeenCalledWith('debug message');
    });

    it('logInfo delegates to logger', () => {
      node.logInfo('info message');
      expect(mockLogger.info).toHaveBeenCalledWith('info message');
    });

    it('logWarn delegates to logger', () => {
      node.logWarn('warn message');
      expect(mockLogger.warn).toHaveBeenCalledWith('warn message');
    });

    it('logError delegates to logger', () => {
      node.logError('error message');
      expect(mockLogger.error).toHaveBeenCalledWith('error message');
    });

    it('logFatal delegates to logger', () => {
      node.logFatal('fatal message');
      expect(mockLogger.fatal).toHaveBeenCalledWith('fatal message');
    });
  });

  describe('createPublisher / createSubscription / createService / createActionServer', () => {
    let node;

    beforeEach(() => {
      node = new Node('test_node', { simDDS: mockSimDDS, logManager: mockLogManager });
    });

    it('createPublisher delegates to SimDDS', () => {
      const pub = node.createPublisher('/topic', 'std_msgs/msg/String');

      expect(mockSimDDS.createPublisher).toHaveBeenCalledWith(
        '/test_node', '/topic', 'std_msgs/msg/String'
      );
      expect(node.publishers).toContain(pub);
    });

    it('createSubscription delegates to SimDDS', () => {
      const callback = vi.fn();
      const sub = node.createSubscription('/topic', 'std_msgs/msg/String', callback);

      expect(mockSimDDS.createSubscription).toHaveBeenCalledWith(
        '/test_node', '/topic', 'std_msgs/msg/String', expect.any(Function)
      );
      expect(node.subscriptions).toContain(sub);
    });

    it('createService delegates to SimDDS', () => {
      const handler = vi.fn();
      const srv = node.createService('/srv', 'test/srv/Test', handler);

      expect(mockSimDDS.createService).toHaveBeenCalledWith(
        '/test_node', '/srv', 'test/srv/Test', expect.any(Function)
      );
      expect(node.services).toContain(srv);
    });

    it('createActionServer delegates to SimDDS', () => {
      const handlers = {
        executeCallback: vi.fn(),
        goalCallback: vi.fn(),
        cancelCallback: vi.fn()
      };
      const action = node.createActionServer('/action', 'test/action/Test', handlers);

      expect(mockSimDDS.createActionServer).toHaveBeenCalledWith(
        '/test_node', '/action', 'test/action/Test', expect.objectContaining({
          executeCallback: expect.any(Function),
          goalCallback: expect.any(Function),
          cancelCallback: expect.any(Function)
        })
      );
      expect(node.actionServers).toContain(action);
    });
  });
});
