import { describe, it, expect, beforeEach, vi } from 'vitest';
import { WorldStateClass } from './WorldState.js';

describe('WorldState', () => {
  let world;

  beforeEach(() => {
    world = new WorldStateClass();
    // Start with no obstacles for most tests
    world.clearObstacles();
  });

  describe('constructor', () => {
    it('initializes with correct dimensions', () => {
      expect(world.width).toBe(11);
      expect(world.height).toBe(11);
    });

    it('initializes with default obstacles', () => {
      const freshWorld = new WorldStateClass();
      expect(freshWorld.obstacles.length).toBeGreaterThan(0);
      // Should have walls
      expect(freshWorld.obstacles.some(o => o.type === 'wall')).toBe(true);
    });
  });

  describe('addObstacle / removeObstacle / clearObstacles', () => {
    it('adds an obstacle', () => {
      world.addObstacle({ type: 'box', x: 2, y: 2, width: 1, height: 1 });
      expect(world.obstacles).toHaveLength(1);
    });

    it('removes an obstacle by index', () => {
      world.addObstacle({ type: 'box', x: 2, y: 2, width: 1, height: 1 });
      world.addObstacle({ type: 'box', x: 4, y: 4, width: 1, height: 1 });

      world.removeObstacle(0);

      expect(world.obstacles).toHaveLength(1);
      expect(world.obstacles[0].x).toBe(4);
    });

    it('handles invalid remove index gracefully', () => {
      world.addObstacle({ type: 'box', x: 2, y: 2, width: 1, height: 1 });

      world.removeObstacle(-1);
      world.removeObstacle(999);

      expect(world.obstacles).toHaveLength(1);
    });

    it('clears all obstacles', () => {
      world.addObstacle({ type: 'box', x: 2, y: 2, width: 1, height: 1 });
      world.addObstacle({ type: 'box', x: 4, y: 4, width: 1, height: 1 });

      world.clearObstacles();

      expect(world.obstacles).toHaveLength(0);
    });
  });

  describe('registerTurtle / updateTurtle / removeTurtle / getTurtle', () => {
    it('registers a turtle', () => {
      world.registerTurtle('turtle1', 5, 5, 0);

      const turtle = world.getTurtle('turtle1');
      expect(turtle).toEqual({ x: 5, y: 5, theta: 0 });
    });

    it('updates a turtle position', () => {
      world.registerTurtle('turtle1', 5, 5, 0);
      world.updateTurtle('turtle1', 6, 7, Math.PI / 2);

      const turtle = world.getTurtle('turtle1');
      expect(turtle).toEqual({ x: 6, y: 7, theta: Math.PI / 2 });
    });

    it('does not update non-existent turtle', () => {
      world.updateTurtle('nonexistent', 1, 1, 0);
      expect(world.getTurtle('nonexistent')).toBeUndefined();
    });

    it('removes a turtle', () => {
      world.registerTurtle('turtle1', 5, 5, 0);
      world.removeTurtle('turtle1');

      expect(world.getTurtle('turtle1')).toBeUndefined();
    });
  });

  describe('checkCollision', () => {
    it('detects collision with left world bound', () => {
      expect(world.checkCollision(0.1, 5, 0.2)).toBe(true);
    });

    it('detects collision with right world bound', () => {
      expect(world.checkCollision(10.9, 5, 0.2)).toBe(true);
    });

    it('detects collision with bottom world bound', () => {
      expect(world.checkCollision(5, 0.1, 0.2)).toBe(true);
    });

    it('detects collision with top world bound', () => {
      expect(world.checkCollision(5, 10.9, 0.2)).toBe(true);
    });

    it('does not detect collision in open space', () => {
      expect(world.checkCollision(5.5, 5.5, 0.2)).toBe(false);
    });

    it('detects collision with obstacle', () => {
      world.addObstacle({ type: 'box', x: 3, y: 3, width: 2, height: 2 });

      // Inside the obstacle
      expect(world.checkCollision(4, 4, 0.2)).toBe(true);
      // On the edge
      expect(world.checkCollision(3.1, 3.1, 0.2)).toBe(true);
    });

    it('does not detect collision near but not touching obstacle', () => {
      world.addObstacle({ type: 'box', x: 3, y: 3, width: 2, height: 2 });

      // Just outside the obstacle (with radius 0.2)
      expect(world.checkCollision(2.5, 2.5, 0.2)).toBe(false);
    });
  });

  describe('_circleRectCollision', () => {
    it('detects collision when circle center inside rect', () => {
      expect(world._circleRectCollision(5, 5, 1, 4, 4, 2, 2)).toBe(true);
    });

    it('detects collision when circle overlaps rect edge', () => {
      // Circle at (3, 5), radius 0.5. Rect from (4, 4) to (6, 6)
      // Closest point on rect is (4, 5), distance = 1, radius = 0.5 -> no collision
      expect(world._circleRectCollision(3, 5, 0.5, 4, 4, 2, 2)).toBe(false);

      // With radius 1.5, should collide
      expect(world._circleRectCollision(3, 5, 1.5, 4, 4, 2, 2)).toBe(true);
    });

    it('detects collision at corner', () => {
      // Circle near corner
      expect(world._circleRectCollision(3.5, 3.5, 1, 4, 4, 2, 2)).toBe(true);
    });

    it('no collision when circle is far from rect', () => {
      expect(world._circleRectCollision(0, 0, 0.5, 4, 4, 2, 2)).toBe(false);
    });
  });

  describe('raycast', () => {
    beforeEach(() => {
      // Add a simple obstacle
      world.addObstacle({ type: 'box', x: 5, y: 5, width: 1, height: 1 });
    });

    it('detects hit on obstacle', () => {
      // Ray from (3, 5.5) pointing right should hit the box
      const result = world.raycast(3, 5.5, 0, 10);

      expect(result.hit).toBe(true);
      expect(result.distance).toBeCloseTo(2, 1);
      expect(result.point.x).toBeCloseTo(5, 1);
    });

    it('returns max range when no hit', () => {
      // Clear obstacles for this test
      world.clearObstacles();

      // Ray from center pointing down (will hit world boundary)
      const result = world.raycast(5.5, 5.5, -Math.PI / 2, 10);

      expect(result.hit).toBe(true);
      expect(result.distance).toBeLessThan(10);
    });

    it('returns correct hit point', () => {
      const result = world.raycast(3, 5.5, 0, 10);

      expect(result.point.x).toBeCloseTo(5, 0.1);
      expect(result.point.y).toBeCloseTo(5.5, 0.1);
    });

    it('respects max range', () => {
      world.clearObstacles();
      const maxRange = 3;
      const result = world.raycast(5.5, 5.5, 0, maxRange);

      expect(result.distance).toBeLessThanOrEqual(maxRange);
    });

    it('handles rays that miss obstacles but hit bounds', () => {
      // Ray pointing away from obstacle toward boundary
      const result = world.raycast(5.5, 5.5, Math.PI, 10); // pointing left

      expect(result.hit).toBe(true);
      expect(result.point.x).toBeLessThan(5.5);
    });
  });

  describe('_rayLineIntersection', () => {
    it('returns intersection for crossing lines', () => {
      // Ray from (0, 0) going right, line segment from (5, -5) to (5, 5)
      const hit = world._rayLineIntersection(0, 0, 1, 0, 5, -5, 5, 5);

      expect(hit).not.toBeNull();
      expect(hit.distance).toBeCloseTo(5, 0.001);
      expect(hit.point.x).toBeCloseTo(5, 0.001);
      expect(hit.point.y).toBeCloseTo(0, 0.001);
    });

    it('returns null for parallel lines', () => {
      // Ray and line both horizontal
      const hit = world._rayLineIntersection(0, 0, 1, 0, 0, 5, 10, 5);

      expect(hit).toBeNull();
    });

    it('returns null when ray points away', () => {
      // Ray pointing left, line is to the right
      const hit = world._rayLineIntersection(0, 0, -1, 0, 5, -5, 5, 5);

      expect(hit).toBeNull();
    });

    it('returns null when intersection is beyond line segment', () => {
      // Line segment from (5, 1) to (5, 2), ray at y=0 won't hit it
      const hit = world._rayLineIntersection(0, 0, 1, 0, 5, 1, 5, 2);

      expect(hit).toBeNull();
    });
  });

  describe('lidarScan', () => {
    beforeEach(() => {
      // Add walls for consistent scanning
      world.addObstacle({ type: 'wall', x: 0, y: 0, width: 11, height: 0.1 });  // Bottom
      world.addObstacle({ type: 'wall', x: 0, y: 10.9, width: 11, height: 0.1 }); // Top
      world.addObstacle({ type: 'wall', x: 0, y: 0, width: 0.1, height: 11 });   // Left
      world.addObstacle({ type: 'wall', x: 10.9, y: 0, width: 0.1, height: 11 }); // Right
    });

    it('returns correct scan structure', () => {
      const scan = world.lidarScan(5.5, 5.5, 0);

      expect(scan).toHaveProperty('angle_min');
      expect(scan).toHaveProperty('angle_max');
      expect(scan).toHaveProperty('angle_increment');
      expect(scan).toHaveProperty('range_min');
      expect(scan).toHaveProperty('range_max');
      expect(scan).toHaveProperty('ranges');
      expect(scan).toHaveProperty('intensities');
    });

    it('returns correct number of rays', () => {
      const numRays = 36;
      const scan = world.lidarScan(5.5, 5.5, 0, { numRays });

      expect(scan.ranges).toHaveLength(numRays);
      expect(scan.intensities).toHaveLength(numRays);
    });

    it('respects range limits', () => {
      const scan = world.lidarScan(5.5, 5.5, 0, {
        rangeMin: 0.5,
        rangeMax: 3.0,
        numRays: 36
      });

      for (const range of scan.ranges) {
        if (range !== Infinity) {
          expect(range).toBeGreaterThanOrEqual(0.5);
          expect(range).toBeLessThanOrEqual(3.0);
        }
      }
    });

    it('detects obstacles', () => {
      world.addObstacle({ type: 'box', x: 7, y: 5, width: 1, height: 1 });

      // Scan from (5.5, 5.5) pointing right should see obstacle
      const scan = world.lidarScan(5.5, 5.5, 0, {
        angleMin: -0.1,
        angleMax: 0.1,
        numRays: 3
      });

      // At least one ray should hit the obstacle
      const minRange = Math.min(...scan.ranges.filter(r => r !== Infinity));
      expect(minRange).toBeLessThan(5); // Obstacle is about 1.5 units away
    });

    it('uses robot heading', () => {
      world.addObstacle({ type: 'box', x: 7, y: 5, width: 1, height: 1 });

      // Same position but facing opposite direction
      const scanRight = world.lidarScan(5.5, 5.5, 0, {
        angleMin: -0.5,
        angleMax: 0.5,
        numRays: 5
      });
      const scanLeft = world.lidarScan(5.5, 5.5, Math.PI, {
        angleMin: -0.5,
        angleMax: 0.5,
        numRays: 5
      });

      // Should see different distances
      const avgRight = scanRight.ranges.filter(r => r !== Infinity).reduce((a, b) => a + b, 0) / 5;
      const avgLeft = scanLeft.ranges.filter(r => r !== Infinity).reduce((a, b) => a + b, 0) / 5;

      expect(avgRight).not.toBeCloseTo(avgLeft, 0);
    });
  });

  describe('findObstacleAt', () => {
    beforeEach(() => {
      world.addObstacle({ type: 'box', x: 3, y: 3, width: 2, height: 2 });
      world.addObstacle({ type: 'wall', x: 0, y: 0, width: 11, height: 0.1 }); // Should be ignored
    });

    it('finds obstacle at point', () => {
      const index = world.findObstacleAt(4, 4);
      expect(index).toBe(0);
    });

    it('returns -1 when no obstacle at point', () => {
      const index = world.findObstacleAt(1, 1);
      expect(index).toBe(-1);
    });

    it('ignores walls', () => {
      const index = world.findObstacleAt(5, 0.05);
      expect(index).toBe(-1);
    });

    it('finds last added obstacle when overlapping', () => {
      world.addObstacle({ type: 'box', x: 3.5, y: 3.5, width: 1, height: 1 });

      // Point inside both obstacles
      const index = world.findObstacleAt(4, 4);
      expect(index).toBe(2); // Last added non-wall
    });
  });

  describe('change notifications', () => {
    it('calls callback on resetObstacles', () => {
      const callback = vi.fn();
      world.setOnChangeCallback(callback);

      world.resetObstacles();

      expect(callback).toHaveBeenCalled();
    });

    it('calls callback on addObstacleAt', () => {
      const callback = vi.fn();
      world.setOnChangeCallback(callback);

      world.addObstacleAt(5, 5);

      expect(callback).toHaveBeenCalled();
    });

    it('calls callback on removeObstacleAt', () => {
      const callback = vi.fn();
      world.addObstacle({ type: 'box', x: 3, y: 3, width: 2, height: 2 });
      world.setOnChangeCallback(callback);

      world.removeObstacleAt(4, 4);

      expect(callback).toHaveBeenCalled();
    });

    it('calls callback on clearNonWallObstacles', () => {
      const callback = vi.fn();
      world.addObstacle({ type: 'box', x: 3, y: 3, width: 2, height: 2 });
      world.setOnChangeCallback(callback);

      world.clearNonWallObstacles();

      expect(callback).toHaveBeenCalled();
    });

    it('calls callback on randomizeObstacles', () => {
      const callback = vi.fn();
      world.setOnChangeCallback(callback);

      world.randomizeObstacles(2);

      expect(callback).toHaveBeenCalled();
    });

    it('clears callback with null', () => {
      const callback = vi.fn();
      world.setOnChangeCallback(callback);
      world.setOnChangeCallback(null);

      // Should dispatch window event instead (or do nothing in test env)
      // Just verify no error thrown
      expect(() => world.resetObstacles()).not.toThrow();
    });
  });

  describe('getObstacleCount', () => {
    it('returns count of non-wall obstacles', () => {
      world.addObstacle({ type: 'wall', x: 0, y: 0, width: 11, height: 0.1 });
      world.addObstacle({ type: 'box', x: 3, y: 3, width: 1, height: 1 });
      world.addObstacle({ type: 'box', x: 5, y: 5, width: 1, height: 1 });

      expect(world.getObstacleCount()).toBe(2);
    });

    it('returns 0 when only walls', () => {
      world.addObstacle({ type: 'wall', x: 0, y: 0, width: 11, height: 0.1 });

      expect(world.getObstacleCount()).toBe(0);
    });
  });

  describe('addObstacleAt', () => {
    it('centers obstacle on given point', () => {
      const index = world.addObstacleAt(5, 5, 2, 2);

      const obs = world.obstacles[index];
      expect(obs.x).toBe(4); // 5 - 2/2
      expect(obs.y).toBe(4); // 5 - 2/2
      expect(obs.width).toBe(2);
      expect(obs.height).toBe(2);
    });

    it('clamps obstacle to world bounds', () => {
      const index = world.addObstacleAt(0.1, 0.1, 1, 1);

      const obs = world.obstacles[index];
      expect(obs.x).toBeGreaterThanOrEqual(0.2);
      expect(obs.y).toBeGreaterThanOrEqual(0.2);
    });
  });

  describe('clearNonWallObstacles', () => {
    it('removes only non-wall obstacles', () => {
      world.addObstacle({ type: 'wall', x: 0, y: 0, width: 11, height: 0.1 });
      world.addObstacle({ type: 'box', x: 3, y: 3, width: 1, height: 1 });
      world.addObstacle({ type: 'box', x: 5, y: 5, width: 1, height: 1 });

      world.clearNonWallObstacles();

      expect(world.obstacles).toHaveLength(1);
      expect(world.obstacles[0].type).toBe('wall');
    });
  });
});
