import { Node } from '../../core/Node.js';
import { Events } from '../../core/Events.js';
import { nodeRegistry } from '../registry.js';

/**
 * SlamNode - Simple occupancy grid mapping for educational purposes
 * Demonstrates core SLAM concepts:
 * - Lidar scan processing
 * - Ray tracing through grid
 * - Occupancy grid updates
 * - Loop closure detection (educational)
 * - Localization against a pre-built map
 */
export class SlamNode extends Node {
  constructor(name = 'slam_node', options = {}) {
    super(name, {
      ...options,
      parameters: {
        map_resolution: 0.1,  // meters per cell
        map_width: 110,       // cells (11m / 0.1m)
        map_height: 110,      // cells
        origin_x: 0.0,        // world X origin
        origin_y: 0.0,        // world Y origin
        hit_prob: 0.9,        // probability of occupied on hit
        miss_prob: 0.3,       // probability of occupied on miss
        // Loop closure parameters
        loop_closure_enabled: true,
        loop_closure_distance: 0.5,     // meters - how close to trigger closure
        loop_closure_min_travel: 3.0,   // meters - minimum travel before checking
        loop_closure_cooldown: 5000,    // ms between detections
        drift_enabled: false,           // odometry drift simulation
        drift_rate: 0.005,              // drift per meter traveled
        drift_max: 0.3,                 // max cumulative drift
        // Localization mode
        localization_mode: false,       // if true, localize against stored map
        ...options.parameters
      }
    });

    // Occupancy grid (log-odds representation)
    this.grid = null;
    this.gridWidth = 0;
    this.gridHeight = 0;
    this.resolution = 0;
    this.originX = 0;
    this.originY = 0;

    // Robot pose (true)
    this.robotX = 5.5;
    this.robotY = 5.5;
    this.robotTheta = 0;

    // Drifted pose (used for mapping when drift is enabled)
    this.driftX = 0;
    this.driftY = 0;
    this.driftTheta = 0;

    // Log-odds constants
    this.l0 = 0;
    this.lOcc = 0;
    this.lFree = 0;

    // Clamping values
    this.lMin = -5;
    this.lMax = 5;

    // Pose graph for loop closure
    this.poseGraph = [];          // Array of { x, y, theta, timestamp, distanceTraveled }
    this.totalDistanceTraveled = 0;
    this.lastPoseX = 5.5;
    this.lastPoseY = 5.5;
    this.lastLoopClosureTime = 0;
    this.loopClosureCount = 0;

    // Stored map for localization mode
    this.storedGrid = null;
  }

  onInit() {
    // Initialize grid
    this._initializeGrid();

    // Calculate log-odds
    const hitProb = this.getParameter('hit_prob');
    const missProb = this.getParameter('miss_prob');
    this.lOcc = Math.log(hitProb / (1 - hitProb));
    this.lFree = Math.log(missProb / (1 - missProb));

    // Subscribe to pose
    this.createSubscription('/turtle1/pose', 'turtlesim/msg/Pose', this._handlePose.bind(this));

    // Subscribe to lidar scan
    this.createSubscription('/scan', 'sensor_msgs/msg/LaserScan', this._handleScan.bind(this));

    // Create map publisher
    this.mapPub = this.createPublisher('/map', 'nav_msgs/msg/OccupancyGrid');

    // Create loop closure publisher
    this.loopClosurePub = this.createPublisher('/loop_closures', 'std_msgs/msg/String');

    // Publish map periodically
    this.createTimer(500, this._publishMap.bind(this)); // 2 Hz

    // Service to save/load map for localization
    this.createService('/slam/save_map', 'std_srvs/srv/Empty', this._handleSaveMap.bind(this));
    this.createService('/slam/load_map', 'std_srvs/srv/Empty', this._handleLoadMap.bind(this));

    // Register parameter validation callback
    this.addOnSetParametersCallback((params) => {
      if ('map_resolution' in params) {
        const v = params.map_resolution;
        if (typeof v !== 'number' || v <= 0 || v > 1.0) {
          return { successful: false, reason: 'map_resolution must be between 0.0 (exclusive) and 1.0' };
        }
      }
      if ('hit_prob' in params) {
        const v = params.hit_prob;
        if (typeof v !== 'number' || v < 0.5 || v > 1.0) {
          return { successful: false, reason: 'hit_prob must be between 0.5 and 1.0' };
        }
      }
      if ('miss_prob' in params) {
        const v = params.miss_prob;
        if (typeof v !== 'number' || v < 0.0 || v > 0.5) {
          return { successful: false, reason: 'miss_prob must be between 0.0 and 0.5' };
        }
      }
      if ('loop_closure_distance' in params) {
        const v = params.loop_closure_distance;
        if (typeof v !== 'number' || v <= 0) {
          return { successful: false, reason: 'loop_closure_distance must be positive' };
        }
      }
      if ('drift_rate' in params) {
        const v = params.drift_rate;
        if (typeof v !== 'number' || v < 0) {
          return { successful: false, reason: 'drift_rate must be non-negative' };
        }
      }
      return { successful: true, reason: '' };
    });

    const mode = this.getParameter('localization_mode') ? 'LOCALIZATION' : 'MAPPING';
    this.logInfo(`SLAM node started in ${mode} mode. Subscribe to /scan and /turtle1/pose`);
    this.logInfo(`Map: ${this.gridWidth}x${this.gridHeight} cells, ${this.resolution}m resolution`);

    if (this.getParameter('loop_closure_enabled')) {
      this.logInfo('Loop closure detection enabled');
    }
    if (this.getParameter('drift_enabled')) {
      this.logInfo(`Odometry drift simulation enabled (rate: ${this.getParameter('drift_rate')})`);
    }

    // Automatically show the map visualization
    window.dispatchEvent(new CustomEvent(Events.SHOW_MAP));
    this.logInfo('Map visualization enabled - watch the occupancy grid build as you explore!');
  }

  _initializeGrid() {
    this.gridWidth = this.getParameter('map_width');
    this.gridHeight = this.getParameter('map_height');
    this.resolution = this.getParameter('map_resolution');
    this.originX = this.getParameter('origin_x');
    this.originY = this.getParameter('origin_y');

    // Initialize grid with unknown (log-odds = 0)
    this.grid = new Float32Array(this.gridWidth * this.gridHeight);
    this.grid.fill(0);
  }

  _handlePose(msg) {
    const prevX = this.robotX;
    const prevY = this.robotY;

    this.robotX = msg.x;
    this.robotY = msg.y;
    this.robotTheta = msg.theta;

    // Track distance traveled
    const dx = this.robotX - prevX;
    const dy = this.robotY - prevY;
    const distStep = Math.sqrt(dx * dx + dy * dy);
    this.totalDistanceTraveled += distStep;

    // Simulate odometry drift
    if (this.getParameter('drift_enabled') && distStep > 0.001) {
      this._simulateDrift(distStep);
    }

    // Add to pose graph periodically (every ~0.5m traveled)
    const distFromLast = Math.sqrt(
      (this.robotX - this.lastPoseX) ** 2 +
      (this.robotY - this.lastPoseY) ** 2
    );
    if (distFromLast >= 0.5) {
      this._addToPoseGraph();
      this.lastPoseX = this.robotX;
      this.lastPoseY = this.robotY;
    }

    // Notify UI of robot pose update (use true pose for display)
    window.dispatchEvent(new CustomEvent(Events.ROBOT_POSE_UPDATE, {
      detail: {
        x: this.robotX,
        y: this.robotY,
        theta: this.robotTheta
      }
    }));
  }

  _simulateDrift(distStep) {
    const rate = this.getParameter('drift_rate');
    const maxDrift = this.getParameter('drift_max');

    // Add small random drift proportional to distance traveled
    this.driftX += (Math.random() - 0.5) * rate * distStep * 2;
    this.driftY += (Math.random() - 0.5) * rate * distStep * 2;
    this.driftTheta += (Math.random() - 0.5) * rate * distStep * 0.5;

    // Clamp drift
    this.driftX = Math.max(-maxDrift, Math.min(maxDrift, this.driftX));
    this.driftY = Math.max(-maxDrift, Math.min(maxDrift, this.driftY));
    this.driftTheta = Math.max(-maxDrift * 0.3, Math.min(maxDrift * 0.3, this.driftTheta));
  }

  _addToPoseGraph() {
    this.poseGraph.push({
      x: this.robotX,
      y: this.robotY,
      theta: this.robotTheta,
      timestamp: Date.now(),
      distanceTraveled: this.totalDistanceTraveled
    });

    // Keep graph manageable
    if (this.poseGraph.length > 1000) {
      this.poseGraph = this.poseGraph.slice(-500);
    }
  }

  _handleScan(msg) {
    // In localization mode, don't update the map
    if (this.getParameter('localization_mode') && this.storedGrid) {
      return;
    }

    // Use drifted pose for mapping (to demonstrate why loop closure matters)
    const mapX = this.robotX + this.driftX;
    const mapY = this.robotY + this.driftY;
    const mapTheta = this.robotTheta + this.driftTheta;

    // Process lidar scan and update occupancy grid
    const { angle_min, angle_increment, ranges, range_max } = msg;

    for (let i = 0; i < ranges.length; i++) {
      const range = ranges[i];
      const angle = mapTheta + angle_min + i * angle_increment;

      if (!isFinite(range) || range <= 0) {
        continue;
      }

      const isHit = range < range_max - 0.1;
      const effectiveRange = isHit ? Math.max(0, range - this.resolution * 0.5) : range;

      const hitX = mapX + Math.cos(angle) * effectiveRange;
      const hitY = mapY + Math.sin(angle) * effectiveRange;

      this._rayTrace(mapX, mapY, hitX, hitY, isHit);
    }

    // Check for loop closure after scan processing
    if (this.getParameter('loop_closure_enabled')) {
      this._checkLoopClosure();
    }
  }

  _checkLoopClosure() {
    const now = Date.now();
    const cooldown = this.getParameter('loop_closure_cooldown');
    const minTravel = this.getParameter('loop_closure_min_travel');
    const closureDist = this.getParameter('loop_closure_distance');

    // Respect cooldown
    if (now - this.lastLoopClosureTime < cooldown) return;

    // Need enough travel to have meaningful loop
    if (this.totalDistanceTraveled < minTravel) return;

    // Need enough pose history
    if (this.poseGraph.length < 10) return;

    // Check if current pose is close to any older pose
    // Skip recent poses (last 10) to avoid false positives
    const searchEnd = this.poseGraph.length - 10;
    for (let i = 0; i < searchEnd; i++) {
      const pose = this.poseGraph[i];
      const dx = this.robotX - pose.x;
      const dy = this.robotY - pose.y;
      const dist = Math.sqrt(dx * dx + dy * dy);

      // Must have traveled significantly since that pose
      const travelSince = this.totalDistanceTraveled - pose.distanceTraveled;
      if (travelSince < minTravel) continue;

      if (dist < closureDist) {
        this._applyLoopClosure(pose);
        this.lastLoopClosureTime = now;
        return;
      }
    }
  }

  _applyLoopClosure(matchedPose) {
    this.loopClosureCount++;

    // Calculate the drift error
    const errorX = this.driftX;
    const errorY = this.driftY;
    const errorTheta = this.driftTheta;

    this.logInfo(`Loop closure #${this.loopClosureCount} detected!`);
    this.logInfo(`  Matched pose: (${matchedPose.x.toFixed(2)}, ${matchedPose.y.toFixed(2)})`);
    this.logInfo(`  Current pose: (${this.robotX.toFixed(2)}, ${this.robotY.toFixed(2)})`);

    if (this.getParameter('drift_enabled')) {
      this.logInfo(`  Drift corrected: dx=${errorX.toFixed(3)}, dy=${errorY.toFixed(3)}, dtheta=${errorTheta.toFixed(3)}`);

      // Reset drift (simulate correction)
      this.driftX = 0;
      this.driftY = 0;
      this.driftTheta = 0;
    }

    // Publish loop closure event
    this.loopClosurePub.publish({
      data: `Loop closure #${this.loopClosureCount} at (${this.robotX.toFixed(2)}, ${this.robotY.toFixed(2)})`
    });

    // Notify UI
    window.dispatchEvent(new CustomEvent(Events.LOOP_CLOSURE_DETECTED, {
      detail: {
        currentPose: { x: this.robotX, y: this.robotY, theta: this.robotTheta },
        matchedPose: { x: matchedPose.x, y: matchedPose.y, theta: matchedPose.theta },
        error: { x: errorX, y: errorY, theta: errorTheta },
        count: this.loopClosureCount
      }
    }));
  }

  /**
   * Save current map for later localization
   */
  _handleSaveMap() {
    this.storedGrid = new Float32Array(this.grid);
    this.logInfo('Map saved for localization. Use ros2 param set /slam_node localization_mode true to switch.');
    return {};
  }

  /**
   * Load stored map for localization
   */
  _handleLoadMap() {
    if (this.storedGrid) {
      this.grid = new Float32Array(this.storedGrid);
      this.logInfo('Stored map loaded into active grid.');
    } else {
      this.logWarn('No stored map available. Save a map first with /slam/save_map.');
    }
    return {};
  }

  /**
   * Bresenham ray tracing through the grid
   */
  _rayTrace(x0, y0, x1, y1, hit) {
    const gx0 = this._worldToGridX(x0);
    const gy0 = this._worldToGridY(y0);
    const gx1 = this._worldToGridX(x1);
    const gy1 = this._worldToGridY(y1);

    const dx = gx1 - gx0;
    const dy = gy1 - gy0;
    const steps = Math.max(Math.abs(dx), Math.abs(dy));

    if (steps === 0) {
      if (gx0 >= 0 && gx0 < this.gridWidth && gy0 >= 0 && gy0 < this.gridHeight) {
        const idx = gy0 * this.gridWidth + gx0;
        if (hit) {
          this.grid[idx] = Math.min(this.lMax, this.grid[idx] + this.lOcc);
        }
      }
      return;
    }

    const xIncrement = dx / steps;
    const yIncrement = dy / steps;

    let x = gx0;
    let y = gy0;
    let lastGridX = -1;
    let lastGridY = -1;

    for (let i = 0; i <= steps; i++) {
      const gridX = Math.round(x);
      const gridY = Math.round(y);

      if (gridX === lastGridX && gridY === lastGridY) {
        x += xIncrement;
        y += yIncrement;
        continue;
      }

      lastGridX = gridX;
      lastGridY = gridY;

      if (gridX >= 0 && gridX < this.gridWidth && gridY >= 0 && gridY < this.gridHeight) {
        const idx = gridY * this.gridWidth + gridX;

        if (i === steps) {
          if (hit) {
            this.grid[idx] = Math.min(this.lMax, this.grid[idx] + this.lOcc);
          }
        } else {
          this.grid[idx] = Math.max(this.lMin, this.grid[idx] + this.lFree);
        }
      }

      x += xIncrement;
      y += yIncrement;
    }
  }

  _worldToGridX(worldX) {
    return Math.floor((worldX - this.originX) / this.resolution);
  }

  _worldToGridY(worldY) {
    return Math.floor((worldY - this.originY) / this.resolution);
  }

  /**
   * Convert log-odds to probability
   */
  _logOddsToProbability(l) {
    return 1 - 1 / (1 + Math.exp(l));
  }

  /**
   * Publish the occupancy grid
   */
  _publishMap() {
    const data = new Int8Array(this.grid.length);

    for (let i = 0; i < this.grid.length; i++) {
      const l = this.grid[i];

      if (Math.abs(l) < 0.1) {
        data[i] = -1;
      } else {
        const prob = this._logOddsToProbability(l);
        data[i] = Math.round(prob * 100);
      }
    }

    const map = {
      header: {
        stamp: this.now(),
        frame_id: 'map'
      },
      info: {
        map_load_time: this.now(),
        resolution: this.resolution,
        width: this.gridWidth,
        height: this.gridHeight,
        origin: {
          position: { x: this.originX, y: this.originY, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
      },
      data: Array.from(data)
    };

    this.mapPub.publish(map);

    // Also notify UI directly
    window.dispatchEvent(new CustomEvent(Events.MAP_UPDATE, {
      detail: {
        map: Array.from(data),
        info: map.info
      }
    }));
  }

  onParameterChange(name, value) {
    if (name === 'hit_prob') {
      this.lOcc = Math.log(value / (1 - value));
    } else if (name === 'miss_prob') {
      this.lFree = Math.log(value / (1 - value));
    } else if (name === 'localization_mode') {
      if (value && this.storedGrid) {
        this.grid = new Float32Array(this.storedGrid);
        this.logInfo('Switched to LOCALIZATION mode - using stored map');
      } else if (value && !this.storedGrid) {
        this.logWarn('No stored map available. Save map first: ros2 service call /slam/save_map std_srvs/srv/Empty');
      } else {
        this.logInfo('Switched to MAPPING mode');
      }
    } else if (name === 'drift_enabled') {
      if (value) {
        this.logInfo('Odometry drift simulation enabled');
      } else {
        this.driftX = 0;
        this.driftY = 0;
        this.driftTheta = 0;
        this.logInfo('Odometry drift simulation disabled, drift reset');
      }
    }
  }

  onShutdown() {
    this.grid = null;
    this.storedGrid = null;
    this.poseGraph = [];
  }
}

// Self-register with the node registry
nodeRegistry.register('simple_slam', 'slam_node', SlamNode);
