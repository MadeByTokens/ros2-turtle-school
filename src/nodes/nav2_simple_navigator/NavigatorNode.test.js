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
    expect(node.getParameter('robot_radius')).toBe(0.6);
  });

  it('has default recovery parameters', () => {
    expect(node.getParameter('recovery_enabled')).toBe(true);
    expect(node.getParameter('stuck_timeout')).toBe(3.0);
    expect(node.getParameter('max_recovery_attempts')).toBe(3);
  });

  it('validates parameter changes', () => {
    node.onInit();

    // Invalid goal_tolerance
    const result1 = node.setParameters({ goal_tolerance: -1 });
    expect(result1.successful).toBe(false);

    // Invalid max_speed
    const result2 = node.setParameters({ max_speed: 0 });
    expect(result2.successful).toBe(false);

    // Invalid robot_radius (negative)
    const result3 = node.setParameters({ robot_radius: -1 });
    expect(result3.successful).toBe(false);

    // Invalid robot_radius (too large)
    const result4 = node.setParameters({ robot_radius: 5.0 });
    expect(result4.successful).toBe(false);

    // Valid changes
    const result5 = node.setParameters({ goal_tolerance: 0.5 });
    expect(result5.successful).toBe(true);

    // Valid robot_radius
    const result6 = node.setParameters({ robot_radius: 1.0 });
    expect(result6.successful).toBe(true);
  });

  it('validates stuck_timeout parameter', () => {
    node.onInit();

    const r1 = node.setParameters({ stuck_timeout: -1 });
    expect(r1.successful).toBe(false);

    const r2 = node.setParameters({ stuck_timeout: 31 });
    expect(r2.successful).toBe(false);

    const r3 = node.setParameters({ stuck_timeout: 5.0 });
    expect(r3.successful).toBe(true);
  });

  it('validates max_recovery_attempts parameter', () => {
    node.onInit();

    const r1 = node.setParameters({ max_recovery_attempts: -1 });
    expect(r1.successful).toBe(false);

    const r2 = node.setParameters({ max_recovery_attempts: 11 });
    expect(r2.successful).toBe(false);

    const r3 = node.setParameters({ max_recovery_attempts: 5 });
    expect(r3.successful).toBe(true);
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

    it('keeps clearance from obstacles when robot_radius > 0', () => {
      // Use a larger map for this test
      const width = 40;
      const height = 40;
      node.mapInfo = {
        width,
        height,
        resolution: 0.5,
        origin: { position: { x: 0, y: 0, z: 0 } },
      };
      node.mapData = new Array(width * height).fill(0);

      // With default robot_radius=0.6 and resolution=0.5, radiusCells=ceil(0.6/0.5)=2
      // Place obstacle at center of the map: grid (20, 20) -> world (10.25, 10.25)
      node.mapData[20 * width + 20] = 100;

      // Path from (3,10.25) to (17,10.25) — same row as obstacle, must route around
      // Keep start/goal well away from edges to avoid inflation hitting boundaries
      const path = node._planPath(3.0, 10.25, 17.0, 10.25);
      expect(path).not.toBeNull();

      // Verify no waypoint is within the inflation radius of the obstacle
      const obstacleWorld = { x: 10.25, y: 10.25 };
      const minClearance = node.getParameter('robot_radius');
      for (const wp of path) {
        const dist = Math.sqrt((wp.x - obstacleWorld.x) ** 2 + (wp.y - obstacleWorld.y) ** 2);
        // Allow tolerance for grid discretization
        expect(dist).toBeGreaterThan(minClearance - node.mapInfo.resolution);
      }
    });

    it('reverts to point-robot behavior when robot_radius is 0', () => {
      // Place obstacle at grid (10, 10)
      node.mapData[10 * 20 + 10] = 100;

      node.setParameters({ robot_radius: 0 });

      // With radius=0, the path can pass adjacent to the obstacle
      const path = node._planPath(1.0, 1.0, 9.0, 1.0);
      expect(path).not.toBeNull();
      expect(path.length).toBeGreaterThan(0);
    });
  });

  describe('_navigateToSinglePose', () => {
    beforeEach(() => {
      node.onInit();
      const width = 20;
      const height = 20;
      node.mapInfo = {
        width,
        height,
        resolution: 0.5,
        origin: { position: { x: 0, y: 0, z: 0 } },
      };
      node.mapData = new Array(width * height).fill(0);
      // Position the robot near the goal for instant success
      node.currentPose = { x: 5.0, y: 5.0, theta: 0 };
      node.running = true;
    });

    it('returns success property for a reachable goal', async () => {
      // The robot pose doesn't update in tests (no turtlesim), so we stop early.
      // Stop the node after a short delay to exit the path-following loop.
      node.setParameters({ recovery_enabled: false });
      setTimeout(() => { node.running = false; }, 200);
      const result = await node._navigateToSinglePose(5.2, 5.2, () => false, vi.fn());
      expect(result).toHaveProperty('success');
    }, 10000);

    it('returns failure when no map available', async () => {
      node.mapData = null;
      const result = await node._navigateToSinglePose(5.0, 5.0, () => false, vi.fn());
      expect(result.success).toBe(false);
    });
  });

  describe('_executeFollowWaypoints', () => {
    beforeEach(() => {
      node.onInit();
      const width = 20;
      const height = 20;
      node.mapInfo = {
        width,
        height,
        resolution: 0.5,
        origin: { position: { x: 0, y: 0, z: 0 } },
      };
      node.mapData = new Array(width * height).fill(0);
      node.currentPose = { x: 5.0, y: 5.0, theta: 0 };
      node.running = true;
    });

    it('returns empty missed list for empty waypoints', async () => {
      const result = await node._executeFollowWaypoints(
        { poses: [] }, 'goal1', vi.fn(), () => false
      );
      expect(result.missed_waypoint_indices).toEqual([]);
    });

    it('reports missed waypoints when path planning fails', async () => {
      // Block goal with obstacle
      for (let gx = 0; gx < 20; gx++) {
        for (let gy = 0; gy < 20; gy++) {
          node.mapData[gy * 20 + gx] = 100;
        }
      }
      // Leave current position free
      node.mapData[10 * 20 + 10] = 0;

      const result = await node._executeFollowWaypoints(
        {
          poses: [{ position: { x: 9.0, y: 9.0 } }]
        },
        'goal1',
        vi.fn(),
        () => false
      );
      expect(result.missed_waypoint_indices).toContain(0);
    });

    it('handles cancellation', async () => {
      let callCount = 0;
      const isCanceled = () => {
        callCount++;
        return callCount > 0; // Cancel immediately
      };

      const result = await node._executeFollowWaypoints(
        {
          poses: [
            { position: { x: 3.0, y: 3.0 } },
            { position: { x: 8.0, y: 8.0 } }
          ]
        },
        'goal1',
        vi.fn(),
        isCanceled
      );
      // Should return with missed indices (cancelled before completing)
      expect(result).toHaveProperty('missed_waypoint_indices');
    });
  });

  describe('_executeRecovery', () => {
    beforeEach(() => {
      node.onInit();
      node.running = true;
    });

    it('publishes velocity commands during recovery', async () => {
      const publishCalls = [];
      node.cmdVelPub = {
        publish: vi.fn((msg) => publishCalls.push(msg)),
        destroy: vi.fn()
      };
      node.recoveryPub = {
        publish: vi.fn(),
        destroy: vi.fn()
      };

      // Speed up by making running false after a short time
      setTimeout(() => { node.running = false; }, 50);

      await node._executeRecovery(1);

      // Should have published angular and/or linear commands
      expect(publishCalls.length).toBeGreaterThan(0);
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
