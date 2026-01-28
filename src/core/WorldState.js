/**
 * WorldState - Manages the simulation world including obstacles,
 * collision detection, and raycasting for simulated sensors
 */
class WorldStateClass {
  constructor() {
    // World dimensions (matches turtlesim: 11x11 units)
    this.width = 11;
    this.height = 11;

    // Obstacles: array of { type, x, y, width, height, ... }
    this.obstacles = [];

    // Turtles managed by turtlesim
    this.turtles = new Map();

    // Default obstacles for SLAM demo
    this._initDefaultObstacles();
  }

  /**
   * Initialize default obstacles for the SLAM demo
   */
  _initDefaultObstacles() {
    this.obstacles = [
      // Outer walls
      { type: 'wall', x: 0, y: 0, width: 11, height: 0.1 },          // Bottom
      { type: 'wall', x: 0, y: 10.9, width: 11, height: 0.1 },       // Top
      { type: 'wall', x: 0, y: 0, width: 0.1, height: 11 },          // Left
      { type: 'wall', x: 10.9, y: 0, width: 0.1, height: 11 },       // Right

      // Interior obstacles for SLAM demo
      { type: 'box', x: 3, y: 3, width: 1.5, height: 1.5 },
      { type: 'box', x: 7, y: 2, width: 1, height: 2 },
      { type: 'box', x: 2, y: 7, width: 2, height: 1 },
      { type: 'box', x: 6, y: 6, width: 1.5, height: 1.5 },
      { type: 'box', x: 8, y: 8, width: 1, height: 1 }
    ];
  }

  /**
   * Add an obstacle
   */
  addObstacle(obstacle) {
    this.obstacles.push(obstacle);
  }

  /**
   * Remove an obstacle by index
   */
  removeObstacle(index) {
    if (index >= 0 && index < this.obstacles.length) {
      this.obstacles.splice(index, 1);
    }
  }

  /**
   * Clear all obstacles
   */
  clearObstacles() {
    this.obstacles = [];
  }

  /**
   * Reset to default obstacles
   */
  resetObstacles() {
    this._initDefaultObstacles();
    this._notifyChange();
  }

  /**
   * Generate random obstacles
   * @param {number} count - Number of obstacles to generate
   * @param {Object} options - Size options
   */
  randomizeObstacles(count = 5, options = {}) {
    const {
      minSize = 0.5,
      maxSize = 2.0,
      keepWalls = true,
      margin = 1.5  // Keep obstacles away from center spawn point
    } = options;

    // Keep walls if requested
    if (keepWalls) {
      this.obstacles = this.obstacles.filter(obs => obs.type === 'wall');
    } else {
      this.obstacles = [];
    }

    // Generate random obstacles
    for (let i = 0; i < count; i++) {
      const width = minSize + Math.random() * (maxSize - minSize);
      const height = minSize + Math.random() * (maxSize - minSize);

      let x, y;
      let attempts = 0;
      const maxAttempts = 50;

      // Try to find a non-overlapping position
      do {
        x = 0.5 + Math.random() * (this.width - width - 1);
        y = 0.5 + Math.random() * (this.height - height - 1);
        attempts++;
      } while (
        attempts < maxAttempts &&
        (this._obstacleOverlaps(x, y, width, height) ||
         this._tooCloseToCenter(x, y, width, height, margin))
      );

      if (attempts < maxAttempts) {
        this.obstacles.push({
          type: 'box',
          x,
          y,
          width,
          height
        });
      }
    }

    this._notifyChange();
  }

  /**
   * Check if a new obstacle would overlap existing ones
   */
  _obstacleOverlaps(x, y, w, h, padding = 0.3) {
    for (const obs of this.obstacles) {
      if (obs.type === 'wall') continue;

      const overlapX = x < obs.x + obs.width + padding && x + w + padding > obs.x;
      const overlapY = y < obs.y + obs.height + padding && y + h + padding > obs.y;

      if (overlapX && overlapY) {
        return true;
      }
    }
    return false;
  }

  /**
   * Check if obstacle is too close to center (turtle spawn point)
   */
  _tooCloseToCenter(x, y, w, h, margin) {
    const centerX = this.width / 2;
    const centerY = this.height / 2;

    // Check if rectangle is within margin of center
    const closestX = Math.max(x, Math.min(centerX, x + w));
    const closestY = Math.max(y, Math.min(centerY, y + h));

    const dx = centerX - closestX;
    const dy = centerY - closestY;

    return Math.sqrt(dx * dx + dy * dy) < margin;
  }

  /**
   * Find obstacle at a given point
   * @returns {number} Index of obstacle, or -1 if none found
   */
  findObstacleAt(x, y) {
    for (let i = this.obstacles.length - 1; i >= 0; i--) {
      const obs = this.obstacles[i];
      if (obs.type === 'wall') continue; // Don't select walls

      if (x >= obs.x && x <= obs.x + obs.width &&
          y >= obs.y && y <= obs.y + obs.height) {
        return i;
      }
    }
    return -1;
  }

  /**
   * Add obstacle at position
   */
  addObstacleAt(x, y, width = 1, height = 1) {
    // Center the obstacle on the click point
    const obs = {
      type: 'box',
      x: x - width / 2,
      y: y - height / 2,
      width,
      height
    };

    // Clamp to world bounds
    obs.x = Math.max(0.2, Math.min(this.width - width - 0.2, obs.x));
    obs.y = Math.max(0.2, Math.min(this.height - height - 0.2, obs.y));

    this.obstacles.push(obs);
    this._notifyChange();
    return this.obstacles.length - 1;
  }

  /**
   * Remove obstacle at position
   * @returns {boolean} True if obstacle was removed
   */
  removeObstacleAt(x, y) {
    const index = this.findObstacleAt(x, y);
    if (index >= 0) {
      this.obstacles.splice(index, 1);
      this._notifyChange();
      return true;
    }
    return false;
  }

  /**
   * Clear all non-wall obstacles
   */
  clearNonWallObstacles() {
    this.obstacles = this.obstacles.filter(obs => obs.type === 'wall');
    this._notifyChange();
  }

  /**
   * Notify listeners of changes
   */
  _notifyChange() {
    window.dispatchEvent(new CustomEvent('world-state-changed'));
  }

  /**
   * Get count of non-wall obstacles
   */
  getObstacleCount() {
    return this.obstacles.filter(obs => obs.type !== 'wall').length;
  }

  /**
   * Register a turtle
   */
  registerTurtle(name, x, y, theta) {
    this.turtles.set(name, { x, y, theta });
  }

  /**
   * Update turtle position
   */
  updateTurtle(name, x, y, theta) {
    if (this.turtles.has(name)) {
      this.turtles.set(name, { x, y, theta });
    }
  }

  /**
   * Remove a turtle
   */
  removeTurtle(name) {
    this.turtles.delete(name);
  }

  /**
   * Get turtle position
   */
  getTurtle(name) {
    return this.turtles.get(name);
  }

  /**
   * Check if a point collides with any obstacle
   * @param {number} x - X coordinate
   * @param {number} y - Y coordinate
   * @param {number} radius - Collision radius (default 0.2)
   * @returns {boolean} True if collision detected
   */
  checkCollision(x, y, radius = 0.2) {
    // Check world bounds
    if (x - radius < 0 || x + radius > this.width ||
        y - radius < 0 || y + radius > this.height) {
      return true;
    }

    // Check obstacles
    for (const obs of this.obstacles) {
      if (this._circleRectCollision(x, y, radius, obs.x, obs.y, obs.width, obs.height)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Circle-rectangle collision detection
   */
  _circleRectCollision(cx, cy, r, rx, ry, rw, rh) {
    // Find closest point on rectangle to circle center
    const closestX = Math.max(rx, Math.min(cx, rx + rw));
    const closestY = Math.max(ry, Math.min(cy, ry + rh));

    // Calculate distance
    const dx = cx - closestX;
    const dy = cy - closestY;
    const distSq = dx * dx + dy * dy;

    return distSq < r * r;
  }

  /**
   * Raycast from a point in a direction
   * @param {number} x - Start X
   * @param {number} y - Start Y
   * @param {number} angle - Ray angle in radians
   * @param {number} maxRange - Maximum ray distance
   * @returns {Object} { hit: boolean, distance: number, point: {x, y} }
   */
  raycast(x, y, angle, maxRange = 10) {
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);

    let closestHit = null;
    let closestDist = maxRange;

    // Check all obstacles
    for (const obs of this.obstacles) {
      const hit = this._rayRectIntersection(x, y, dx, dy, obs.x, obs.y, obs.width, obs.height);
      if (hit && hit.distance < closestDist) {
        closestDist = hit.distance;
        closestHit = hit;
      }
    }

    // Check world bounds
    const boundHits = [
      this._rayLineIntersection(x, y, dx, dy, 0, 0, this.width, 0),          // Bottom
      this._rayLineIntersection(x, y, dx, dy, 0, this.height, this.width, this.height), // Top
      this._rayLineIntersection(x, y, dx, dy, 0, 0, 0, this.height),          // Left
      this._rayLineIntersection(x, y, dx, dy, this.width, 0, this.width, this.height)   // Right
    ];

    for (const hit of boundHits) {
      if (hit && hit.distance > 0 && hit.distance < closestDist) {
        closestDist = hit.distance;
        closestHit = hit;
      }
    }

    if (closestHit) {
      return {
        hit: true,
        distance: closestDist,
        point: closestHit.point
      };
    }

    return {
      hit: false,
      distance: maxRange,
      point: { x: x + dx * maxRange, y: y + dy * maxRange }
    };
  }

  /**
   * Ray-rectangle intersection
   */
  _rayRectIntersection(rx, ry, dx, dy, bx, by, bw, bh) {
    // Check all four edges of the rectangle
    const edges = [
      { x1: bx, y1: by, x2: bx + bw, y2: by },           // Bottom
      { x1: bx, y1: by + bh, x2: bx + bw, y2: by + bh }, // Top
      { x1: bx, y1: by, x2: bx, y2: by + bh },           // Left
      { x1: bx + bw, y1: by, x2: bx + bw, y2: by + bh }  // Right
    ];

    let closest = null;
    let closestDist = Infinity;

    for (const edge of edges) {
      const hit = this._rayLineIntersection(rx, ry, dx, dy, edge.x1, edge.y1, edge.x2, edge.y2);
      if (hit && hit.distance > 0.001 && hit.distance < closestDist) {
        closestDist = hit.distance;
        closest = hit;
      }
    }

    return closest;
  }

  /**
   * Ray-line segment intersection
   */
  _rayLineIntersection(rx, ry, dx, dy, x1, y1, x2, y2) {
    const sx = x2 - x1;
    const sy = y2 - y1;

    const denom = dx * sy - dy * sx;
    if (Math.abs(denom) < 0.0001) return null; // Parallel

    const t = ((x1 - rx) * sy - (y1 - ry) * sx) / denom;
    const u = ((x1 - rx) * dy - (y1 - ry) * dx) / denom;

    if (t > 0 && u >= 0 && u <= 1) {
      return {
        distance: t,
        point: { x: rx + dx * t, y: ry + dy * t }
      };
    }

    return null;
  }

  /**
   * Perform a lidar scan
   * @param {number} x - Robot X position
   * @param {number} y - Robot Y position
   * @param {number} theta - Robot heading
   * @param {Object} config - Lidar configuration
   * @returns {Object} LaserScan-like data
   */
  lidarScan(x, y, theta, config = {}) {
    const {
      angleMin = -Math.PI,           // Start angle
      angleMax = Math.PI,            // End angle
      angleIncrement = Math.PI / 180, // 1 degree resolution
      rangeMin = 0.1,
      rangeMax = 10.0,
      numRays = 360
    } = config;

    const ranges = [];
    const intensities = [];
    const actualIncrement = (angleMax - angleMin) / numRays;

    for (let i = 0; i < numRays; i++) {
      const angle = theta + angleMin + i * actualIncrement;
      const result = this.raycast(x, y, angle, rangeMax);

      if (result.hit && result.distance >= rangeMin) {
        ranges.push(result.distance);
        intensities.push(1.0);
      } else if (result.distance < rangeMin) {
        ranges.push(Infinity); // Too close
        intensities.push(0);
      } else {
        ranges.push(Infinity); // No hit
        intensities.push(0);
      }
    }

    return {
      angle_min: angleMin,
      angle_max: angleMax,
      angle_increment: actualIncrement,
      time_increment: 0,
      scan_time: 0.1,
      range_min: rangeMin,
      range_max: rangeMax,
      ranges,
      intensities
    };
  }

  /**
   * Get all obstacles for rendering
   */
  getObstacles() {
    return this.obstacles;
  }
}

// Singleton instance
export const WorldState = new WorldStateClass();
