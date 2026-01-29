import { describe, it, expect, beforeEach, vi, afterEach } from 'vitest';

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
    createActionServer: vi.fn((nodeId, name, type, handlers) => {
      return { id: 'act1', name, handlers, destroy: vi.fn() };
    }),
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

import { NavigatorNode } from './NavigatorNode.js';

describe('NavigatorNode', () => {
  let node;

  beforeEach(() => {
    node = new NavigatorNode('navigator_node', {});
  });

  it('has default parameters', () => {
    expect(node.getParameter('goal_tolerance')).toBe(0.3);
    expect(node.getParameter('max_speed')).toBe(1.5);
    expect(node.getParameter('obstacle_threshold')).toBe(50);
  });

  it('validates parameter changes', () => {
    node.onInit();

    // Invalid goal_tolerance
    const result1 = node.setParameters({ goal_tolerance: -1 });
    expect(result1.successful).toBe(false);

    // Invalid max_speed
    const result2 = node.setParameters({ max_speed: 0 });
    expect(result2.successful).toBe(false);

    // Valid changes
    const result3 = node.setParameters({ goal_tolerance: 0.5 });
    expect(result3.successful).toBe(true);
  });

  describe('A* path planning', () => {
    beforeEach(() => {
      node.onInit();
      // Set up a simple map
      const width = 20;
      const height = 20;
      node.mapInfo = {
        width,
        height,
        resolution: 0.5,
        origin: { position: { x: 0, y: 0, z: 0 } },
      };
      // All cells free (value 0)
      node.mapData = new Array(width * height).fill(0);
    });

    it('finds path in open space', () => {
      const path = node._planPath(1.0, 1.0, 5.0, 5.0);
      expect(path).not.toBeNull();
      expect(path.length).toBeGreaterThan(0);

      // First waypoint should be near start
      expect(path[0].x).toBeCloseTo(1.25, 0);
      expect(path[0].y).toBeCloseTo(1.25, 0);

      // Last waypoint should be near goal
      const last = path[path.length - 1];
      expect(last.x).toBeCloseTo(5.25, 0);
      expect(last.y).toBeCloseTo(5.25, 0);
    });

    it('returns null when goal is in obstacle', () => {
      // Place obstacle at goal (grid cell 10,10 -> world 5.25, 5.25)
      const gx = 10;
      const gy = 10;
      node.mapData[gy * 20 + gx] = 100;

      const path = node._planPath(1.0, 1.0, 5.25, 5.25);
      expect(path).toBeNull();
    });

    it('navigates around obstacles', () => {
      // Create a wall of obstacles in the middle (x=4 to x=6, y=4)
      for (let gx = 8; gx <= 12; gx++) {
        node.mapData[8 * 20 + gx] = 100;
      }

      const path = node._planPath(1.0, 1.0, 8.0, 1.0);
      expect(path).not.toBeNull();
      expect(path.length).toBeGreaterThan(0);
    });
  });

  describe('proportional controller', () => {
    it('computes velocity toward target', () => {
      node.onInit();
      const cmd = node._computeControl(
        { x: 1, y: 1, theta: 0 },
        { x: 3, y: 1 }
      );
      expect(cmd.linear.x).toBeGreaterThan(0);
      expect(Math.abs(cmd.angular.z)).toBeLessThan(0.5); // Nearly straight
    });

    it('turns when target is to the side', () => {
      node.onInit();
      const cmd = node._computeControl(
        { x: 1, y: 1, theta: 0 },
        { x: 1, y: 3 } // Target is 90 degrees left
      );
      expect(cmd.angular.z).toBeGreaterThan(0); // Turn left
    });
  });

  describe('helper methods', () => {
    it('computes distance correctly', () => {
      expect(node._distance({ x: 0, y: 0 }, { x: 3, y: 4 })).toBeCloseTo(5.0);
    });

    it('computes path distance from index', () => {
      const path = [
        { x: 0, y: 0 },
        { x: 1, y: 0 },
        { x: 2, y: 0 },
        { x: 3, y: 0 },
      ];
      expect(node._pathDistanceFrom(path, 0)).toBeCloseTo(3.0);
      expect(node._pathDistanceFrom(path, 2)).toBeCloseTo(1.0);
    });
  });
});
