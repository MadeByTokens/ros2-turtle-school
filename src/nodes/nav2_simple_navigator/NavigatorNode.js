import { Node } from '../../core/Node.js';
import { Events } from '../../core/Events.js';
import { nodeRegistry } from '../registry.js';

/**
 * NavigatorNode - Simple Nav2-style path planner for educational purposes
 * Demonstrates: A* path planning, action servers, autonomous navigation
 *
 * Usage:
 *   ros2 run nav2_simple_navigator navigator_node
 *   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 8.0, y: 8.0}}}"
 */
export class NavigatorNode extends Node {
  constructor(name = 'navigator_node', options = {}) {
    super(name, {
      ...options,
      parameters: {
        goal_tolerance: 0.3,
        max_speed: 1.5,
        max_angular_speed: 2.5,
        obstacle_threshold: 50,
        robot_radius: 0.6,
        ...options.parameters
      }
    });

    this.mapData = null;
    this.mapInfo = null;
    this._inflationRadiusCells = 0;
    this.currentPose = { x: 5.5, y: 5.5, theta: 0 };
    this.isNavigating = false;
  }

  onInit() {
    this.createSubscription('/map', 'nav_msgs/msg/OccupancyGrid', this._handleMap.bind(this));
    this.createSubscription('/turtle1/pose', 'turtlesim/msg/Pose', this._handlePose.bind(this));

    this.pathPub = this.createPublisher('/plan', 'nav_msgs/msg/Path');
    this.cmdVelPub = this.createPublisher('/turtle1/cmd_vel', 'geometry_msgs/msg/Twist');

    this.createActionServer('/navigate_to_pose', 'nav2_msgs/action/NavigateToPose', {
      executeCallback: this._executeNavigate.bind(this),
      goalCallback: () => true
    });

    this.addOnSetParametersCallback((params) => {
      if ('goal_tolerance' in params && (params.goal_tolerance <= 0 || params.goal_tolerance > 5.0)) {
        return { successful: false, reason: 'goal_tolerance must be between 0 (exclusive) and 5.0' };
      }
      if ('max_speed' in params && (params.max_speed <= 0 || params.max_speed > 10.0)) {
        return { successful: false, reason: 'max_speed must be between 0 (exclusive) and 10.0' };
      }
      if ('obstacle_threshold' in params && (params.obstacle_threshold < 0 || params.obstacle_threshold > 100)) {
        return { successful: false, reason: 'obstacle_threshold must be between 0 and 100' };
      }
      if ('robot_radius' in params && (params.robot_radius < 0 || params.robot_radius > 3.0)) {
        return { successful: false, reason: 'robot_radius must be between 0 and 3.0' };
      }
      return { successful: true, reason: '' };
    });

    this.logInfo('Navigator node started. Waiting for map and goals...');
    this.logInfo('Send goals: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {position: {x: 8, y: 8}}}"');
  }

  _handleMap(msg) {
    this.mapInfo = msg.info;
    this.mapData = msg.data;
    this._updateInflationRadius();
  }

  _updateInflationRadius() {
    if (!this.mapInfo) return;
    const radius = this.getParameter('robot_radius');
    this._inflationRadiusCells = radius > 0 ? Math.ceil(radius / this.mapInfo.resolution) : 0;
  }

  _handlePose(msg) {
    this.currentPose = { x: msg.x, y: msg.y, theta: msg.theta };
  }

  async _executeNavigate(goal, goalId, feedbackCb, isCanceled) {
    const targetX = goal.pose?.pose?.position?.x ?? goal.pose?.position?.x ?? 0;
    const targetY = goal.pose?.pose?.position?.y ?? goal.pose?.position?.y ?? 0;

    this.logInfo(`Received navigation goal: (${targetX.toFixed(2)}, ${targetY.toFixed(2)})`);

    if (!this.mapData || !this.mapInfo) {
      this.logError('No map available. Start SLAM node first: ros2 run simple_slam slam_node');
      return {};
    }

    // Plan path using A*
    const path = this._planPath(this.currentPose.x, this.currentPose.y, targetX, targetY);
    if (!path) {
      this.logError('Failed to find path to goal. Target may be in an obstacle or unreachable.');
      return {};
    }

    this.logInfo(`Path planned with ${path.length} waypoints`);
    this._publishPath(path);
    this._visualizePath(path);

    // Follow path
    this.isNavigating = true;
    const tolerance = this.getParameter('goal_tolerance');
    let waypointIdx = 0;

    // Skip waypoints that are very close together for smoother driving
    const step = Math.max(1, Math.floor(path.length / 50));

    while (waypointIdx < path.length && this.running) {
      if (isCanceled()) {
        this._stopRobot();
        this.logInfo('Navigation cancelled');
        this.isNavigating = false;
        this._clearPath();
        return {};
      }

      const target = path[waypointIdx];
      const dist = this._distance(this.currentPose, target);

      // Send feedback
      feedbackCb({
        current_pose: {
          header: { stamp: this.now(), frame_id: 'map' },
          pose: {
            position: { x: this.currentPose.x, y: this.currentPose.y, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
          }
        },
        distance_remaining: this._pathDistanceFrom(path, waypointIdx)
      });

      if (dist < tolerance) {
        waypointIdx += step;
        continue;
      }

      // Proportional controller
      const cmd = this._computeControl(this.currentPose, target);
      this.cmdVelPub.publish(cmd);

      await this._sleep(100);
    }

    this._stopRobot();
    this.isNavigating = false;
    this._clearPath();

    const finalDist = this._distance(this.currentPose, { x: targetX, y: targetY });
    if (finalDist < tolerance * 2) {
      this.logInfo(`Goal reached! Final distance: ${finalDist.toFixed(2)}m`);
    } else {
      this.logWarn(`Navigation ended. Distance to goal: ${finalDist.toFixed(2)}m`);
    }

    return {};
  }

  /**
   * A* path planning on occupancy grid
   */
  _planPath(startX, startY, goalX, goalY) {
    const threshold = this.getParameter('obstacle_threshold');
    this._updateInflationRadius();
    const radiusCells = this._inflationRadiusCells;
    const startG = this._worldToGrid(startX, startY);
    const goalG = this._worldToGrid(goalX, goalY);

    if (!this._isGridFreeInflated(goalG.gx, goalG.gy, threshold, radiusCells)) {
      this.logWarn('Goal position is inside an obstacle');
      return null;
    }

    const key = (gx, gy) => `${gx},${gy}`;
    const startKey = key(startG.gx, startG.gy);
    const goalKey = key(goalG.gx, goalG.gy);

    const openSet = new MinHeap();
    const gScore = new Map();
    const cameFrom = new Map();
    const closedSet = new Set();

    gScore.set(startKey, 0);
    openSet.push(this._heuristic(startG, goalG), startKey);

    const neighbors = [[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]];
    let iterations = 0;
    const maxIterations = 50000;

    while (!openSet.isEmpty() && iterations < maxIterations) {
      iterations++;
      const currentKey = openSet.pop();

      if (closedSet.has(currentKey)) continue;
      closedSet.add(currentKey);

      if (currentKey === goalKey) {
        return this._reconstructPath(cameFrom, currentKey);
      }

      const [cx, cy] = currentKey.split(',').map(Number);

      for (const [dx, dy] of neighbors) {
        const nx = cx + dx;
        const ny = cy + dy;

        if (!this._isGridFreeInflated(nx, ny, threshold, radiusCells)) continue;

        const nKey = key(nx, ny);
        const moveCost = (dx !== 0 && dy !== 0) ? 1.414 : 1.0;
        const tentativeG = gScore.get(currentKey) + moveCost;

        if (!gScore.has(nKey) || tentativeG < gScore.get(nKey)) {
          cameFrom.set(nKey, currentKey);
          gScore.set(nKey, tentativeG);
          const f = tentativeG + this._heuristic({ gx: nx, gy: ny }, goalG);
          openSet.push(f, nKey);
        }
      }
    }

    return null;
  }

  _heuristic(from, to) {
    const dx = to.gx - from.gx;
    const dy = to.gy - from.gy;
    return Math.sqrt(dx * dx + dy * dy);
  }

  _isGridFree(gx, gy, threshold) {
    if (!this.mapInfo) return false;
    if (gx < 0 || gx >= this.mapInfo.width || gy < 0 || gy >= this.mapInfo.height) return false;
    const idx = gy * this.mapInfo.width + gx;
    const value = this.mapData[idx];
    return value >= 0 && value < threshold;
  }

  _isGridFreeInflated(gx, gy, threshold, radiusCells) {
    if (radiusCells <= 0) return this._isGridFree(gx, gy, threshold);
    const r2 = radiusCells * radiusCells;
    for (let dx = -radiusCells; dx <= radiusCells; dx++) {
      for (let dy = -radiusCells; dy <= radiusCells; dy++) {
        if (dx * dx + dy * dy > r2) continue;
        if (!this._isGridFree(gx + dx, gy + dy, threshold)) return false;
      }
    }
    return true;
  }

  _worldToGrid(x, y) {
    const gx = Math.floor((x - this.mapInfo.origin.position.x) / this.mapInfo.resolution);
    const gy = Math.floor((y - this.mapInfo.origin.position.y) / this.mapInfo.resolution);
    return { gx, gy };
  }

  _gridToWorld(gx, gy) {
    const x = gx * this.mapInfo.resolution + this.mapInfo.origin.position.x + this.mapInfo.resolution / 2;
    const y = gy * this.mapInfo.resolution + this.mapInfo.origin.position.y + this.mapInfo.resolution / 2;
    return { x, y };
  }

  _reconstructPath(cameFrom, goalKey) {
    const path = [];
    let current = goalKey;
    while (current) {
      const [gx, gy] = current.split(',').map(Number);
      path.unshift(this._gridToWorld(gx, gy));
      current = cameFrom.get(current);
    }
    return path;
  }

  _computeControl(current, target) {
    const dx = target.x - current.x;
    const dy = target.y - current.y;
    const distance = Math.sqrt(dx * dx + dy * dy);
    const targetAngle = Math.atan2(dy, dx);

    let angleError = targetAngle - current.theta;
    while (angleError > Math.PI) angleError -= 2 * Math.PI;
    while (angleError < -Math.PI) angleError += 2 * Math.PI;

    const maxSpeed = this.getParameter('max_speed');
    const maxAngular = this.getParameter('max_angular_speed');

    // Slow down when turning sharply
    const turnFactor = Math.max(0.1, 1.0 - Math.abs(angleError) / Math.PI);
    let linear = Math.min(maxSpeed, 0.8 * distance) * turnFactor;
    let angular = Math.max(-maxAngular, Math.min(maxAngular, 2.5 * angleError));

    return {
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    };
  }

  _stopRobot() {
    this.cmdVelPub.publish({
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 }
    });
  }

  _publishPath(path) {
    this.pathPub.publish({
      header: { stamp: this.now(), frame_id: 'map' },
      poses: path.map(p => ({
        header: { stamp: this.now(), frame_id: 'map' },
        pose: {
          position: { x: p.x, y: p.y, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 }
        }
      }))
    });
  }

  _visualizePath(path) {
    window.dispatchEvent(new CustomEvent(Events.PATH_PLANNED, {
      detail: { path }
    }));
  }

  _clearPath() {
    window.dispatchEvent(new CustomEvent(Events.PATH_CLEARED));
  }

  _distance(p1, p2) {
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  _pathDistanceFrom(path, fromIndex) {
    let dist = 0;
    for (let i = fromIndex; i < path.length - 1; i++) {
      dist += this._distance(path[i], path[i + 1]);
    }
    return dist;
  }

  _sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  onShutdown() {
    this._stopRobot();
    this._clearPath();
    this.isNavigating = false;
  }
}

/**
 * MinHeap for A* priority queue
 */
class MinHeap {
  constructor() {
    this.heap = [];
  }

  push(priority, value) {
    this.heap.push({ priority, value });
    this._bubbleUp(this.heap.length - 1);
  }

  pop() {
    if (this.heap.length === 0) return null;
    const min = this.heap[0];
    const last = this.heap.pop();
    if (this.heap.length > 0) {
      this.heap[0] = last;
      this._bubbleDown(0);
    }
    return min.value;
  }

  isEmpty() {
    return this.heap.length === 0;
  }

  _bubbleUp(i) {
    while (i > 0) {
      const parent = Math.floor((i - 1) / 2);
      if (this.heap[i].priority >= this.heap[parent].priority) break;
      [this.heap[i], this.heap[parent]] = [this.heap[parent], this.heap[i]];
      i = parent;
    }
  }

  _bubbleDown(i) {
    while (true) {
      let min = i;
      const left = 2 * i + 1;
      const right = 2 * i + 2;
      if (left < this.heap.length && this.heap[left].priority < this.heap[min].priority) min = left;
      if (right < this.heap.length && this.heap[right].priority < this.heap[min].priority) min = right;
      if (min === i) break;
      [this.heap[i], this.heap[min]] = [this.heap[min], this.heap[i]];
      i = min;
    }
  }
}

nodeRegistry.register('nav2_simple_navigator', 'navigator_node', NavigatorNode);
