import { Node } from '../../core/Node.js';

/**
 * SlamNode - Simple occupancy grid mapping for educational purposes
 * Demonstrates core SLAM concepts:
 * - Lidar scan processing
 * - Ray tracing through grid
 * - Occupancy grid updates
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

    // Robot pose
    this.robotX = 5.5;
    this.robotY = 5.5;
    this.robotTheta = 0;

    // Log-odds constants
    this.l0 = 0;  // prior (unknown)
    this.lOcc = 0;  // calculated from hit_prob
    this.lFree = 0; // calculated from miss_prob

    // Clamping values
    this.lMin = -5;
    this.lMax = 5;
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

    // Publish map periodically
    this.createTimer(500, this._publishMap.bind(this)); // 2 Hz

    this.logInfo('SLAM node started. Subscribe to /scan and /turtle1/pose');
    this.logInfo(`Map: ${this.gridWidth}x${this.gridHeight} cells, ${this.resolution}m resolution`);

    // Automatically show the map visualization
    window.dispatchEvent(new CustomEvent('show-map'));
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
    this.robotX = msg.x;
    this.robotY = msg.y;
    this.robotTheta = msg.theta;

    // Notify UI of robot pose update
    window.dispatchEvent(new CustomEvent('robot-pose-update', {
      detail: {
        x: this.robotX,
        y: this.robotY,
        theta: this.robotTheta
      }
    }));
  }

  _handleScan(msg) {
    // Process lidar scan and update occupancy grid
    const { angle_min, angle_increment, ranges, range_max } = msg;

    for (let i = 0; i < ranges.length; i++) {
      const range = ranges[i];
      const angle = this.robotTheta + angle_min + i * angle_increment;

      if (!isFinite(range) || range <= 0) {
        continue;
      }

      // End point of ray
      const hitX = this.robotX + Math.cos(angle) * range;
      const hitY = this.robotY + Math.sin(angle) * range;

      // Ray trace from robot to hit point
      this._rayTrace(this.robotX, this.robotY, hitX, hitY, range < range_max - 0.1);
    }
  }

  /**
   * Bresenham ray tracing through the grid
   */
  _rayTrace(x0, y0, x1, y1, hit) {
    // Convert to grid coordinates
    const gx0 = this._worldToGridX(x0);
    const gy0 = this._worldToGridY(y0);
    const gx1 = this._worldToGridX(x1);
    const gy1 = this._worldToGridY(y1);

    // Bresenham's line algorithm
    let x = gx0;
    let y = gy0;
    const dx = Math.abs(gx1 - gx0);
    const dy = Math.abs(gy1 - gy0);
    const sx = gx0 < gx1 ? 1 : -1;
    const sy = gy0 < gy1 ? 1 : -1;
    let err = dx - dy;

    while (true) {
      // Check bounds
      if (x >= 0 && x < this.gridWidth && y >= 0 && y < this.gridHeight) {
        const idx = y * this.gridWidth + x;

        // Is this the endpoint?
        if (x === gx1 && y === gy1) {
          if (hit) {
            // Mark as occupied
            this.grid[idx] = Math.min(this.lMax, this.grid[idx] + this.lOcc);
          }
          break;
        } else {
          // Mark as free (ray passed through)
          this.grid[idx] = Math.max(this.lMin, this.grid[idx] + this.lFree);
        }
      }

      if (x === gx1 && y === gy1) break;

      const e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
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
    // Convert log-odds grid to occupancy values (-1, 0-100)
    const data = new Int8Array(this.grid.length);

    for (let i = 0; i < this.grid.length; i++) {
      const l = this.grid[i];

      if (Math.abs(l) < 0.1) {
        // Unknown
        data[i] = -1;
      } else {
        // Convert to probability and then to 0-100
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
    window.dispatchEvent(new CustomEvent('map-update', {
      detail: {
        map: Array.from(data),
        info: map.info
      }
    }));
  }

  onShutdown() {
    this.grid = null;
  }
}
