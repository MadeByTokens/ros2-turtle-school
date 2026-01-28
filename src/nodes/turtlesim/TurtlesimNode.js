import { Node } from '../../core/Node.js';
import { WorldState } from '../../core/WorldState.js';
import { nodeRegistry } from '../registry.js';

/**
 * TurtlesimNode - Main turtlesim simulation node
 * Renders turtles on canvas, handles physics, and provides ROS2 interface
 */
export class TurtlesimNode extends Node {
  constructor(name = 'turtlesim', options = {}) {
    super(name, {
      ...options,
      parameters: {
        background_r: 69,
        background_g: 86,
        background_b: 255,
        ...options.parameters
      }
    });

    // Turtles in the simulation
    this.turtles = new Map();

    // Canvas reference (set by Canvas.js)
    this.canvas = null;
    this.ctx = null;

    // Turtle image
    this.turtleImage = null;
    this.turtleImageLoaded = false;

    // Pen trails
    this.trails = [];

    // Physics update rate
    this.updateRate = 60; // Hz
    this.publishRate = 10; // Hz for pose
    this.lastPosePublish = 0;

    // Lidar config for SLAM
    this.lidarEnabled = true;
    this.lidarPublishRate = 10; // Hz
    this.lastLidarPublish = 0;
  }

  onInit() {
    // Create default turtle
    this._spawnTurtle('turtle1', 5.544445, 5.544445, 0);

    // Register services
    this.createService('/spawn', 'turtlesim/srv/Spawn', this._handleSpawn.bind(this));
    this.createService('/kill', 'turtlesim/srv/Kill', this._handleKill.bind(this));
    this.createService('/clear', 'turtlesim/srv/Empty', this._handleClear.bind(this));
    this.createService('/reset', 'turtlesim/srv/Empty', this._handleReset.bind(this));

    // Start update loop
    this.createTimer(1000 / this.updateRate, this._update.bind(this));

    // Notify canvas to render
    this._notifyCanvas();

    this.logInfo('Turtlesim node started');
  }

  onShutdown() {
    // Clear all turtles and their active action intervals
    for (const [name, turtle] of this.turtles) {
      // Clear any active action interval to prevent memory leak
      if (turtle.activeActionInterval) {
        clearInterval(turtle.activeActionInterval);
        turtle.activeActionInterval = null;
      }
      WorldState.removeTurtle(name);
    }
    this.turtles.clear();
    this.trails = [];
  }

  onParameterChange(name, value) {
    if (name.startsWith('background_')) {
      // Trigger canvas redraw
      this._notifyCanvas();
    }
  }

  /**
   * Spawn a new turtle
   */
  _spawnTurtle(name, x, y, theta) {
    if (this.turtles.has(name)) {
      return null;
    }

    const turtle = {
      name,
      x,
      y,
      theta,
      linearVelocity: 0,
      angularVelocity: 0,
      pen: {
        r: 184,
        g: 184,
        b: 184,
        width: 3,
        off: false
      },
      // Track active action interval for cleanup
      activeActionInterval: null
    };

    this.turtles.set(name, turtle);
    WorldState.registerTurtle(name, x, y, theta);

    // Create turtle-specific publishers and subscribers
    const prefix = `/${name}`;

    turtle.posePub = this.createPublisher(`${prefix}/pose`, 'turtlesim/msg/Pose');
    turtle.colorPub = this.createPublisher(`${prefix}/color_sensor`, 'turtlesim/msg/Color');

    turtle.cmdVelSub = this.createSubscription(
      `${prefix}/cmd_vel`,
      'geometry_msgs/msg/Twist',
      (msg) => this._handleCmdVel(name, msg)
    );

    // Create turtle-specific services
    turtle.setPenService = this.createService(
      `${prefix}/set_pen`,
      'turtlesim/srv/SetPen',
      (req) => this._handleSetPen(name, req)
    );

    turtle.teleportAbsService = this.createService(
      `${prefix}/teleport_absolute`,
      'turtlesim/srv/TeleportAbsolute',
      (req) => this._handleTeleportAbsolute(name, req)
    );

    turtle.teleportRelService = this.createService(
      `${prefix}/teleport_relative`,
      'turtlesim/srv/TeleportRelative',
      (req) => this._handleTeleportRelative(name, req)
    );

    // Create rotate absolute action for first turtle
    if (name === 'turtle1') {
      turtle.rotateAbsAction = this.createActionServer(
        `${prefix}/rotate_absolute`,
        'turtlesim/action/RotateAbsolute',
        {
          executeCallback: (goal, goalId, feedbackCb, isCanceled) =>
            this._executeRotateAbsolute(name, goal, feedbackCb, isCanceled),
          goalCallback: () => true
        }
      );

      // Create lidar publisher for SLAM demo
      if (this.lidarEnabled) {
        turtle.scanPub = this.createPublisher('/scan', 'sensor_msgs/msg/LaserScan');
      }
    }

    this.logInfo(`Spawned turtle '${name}' at (${x.toFixed(2)}, ${y.toFixed(2)})`);
    return name;
  }

  /**
   * Handle /spawn service
   */
  _handleSpawn(request) {
    let name = request.name;
    if (!name) {
      // Generate unique name
      let i = 1;
      while (this.turtles.has(`turtle${i}`)) i++;
      name = `turtle${i}`;
    }

    const result = this._spawnTurtle(name, request.x, request.y, request.theta);
    return { name: result || '' };
  }

  /**
   * Handle /kill service
   */
  _handleKill(request) {
    const turtle = this.turtles.get(request.name);
    if (!turtle) {
      throw new Error(`Turtle '${request.name}' not found`);
    }

    // Clear any active action interval to prevent memory leak
    if (turtle.activeActionInterval) {
      clearInterval(turtle.activeActionInterval);
      turtle.activeActionInterval = null;
    }

    // Cleanup turtle resources
    if (turtle.posePub) turtle.posePub.destroy();
    if (turtle.colorPub) turtle.colorPub.destroy();
    if (turtle.cmdVelSub) turtle.cmdVelSub.destroy();
    if (turtle.setPenService) turtle.setPenService.destroy();
    if (turtle.teleportAbsService) turtle.teleportAbsService.destroy();
    if (turtle.teleportRelService) turtle.teleportRelService.destroy();
    if (turtle.rotateAbsAction) turtle.rotateAbsAction.destroy();
    if (turtle.scanPub) turtle.scanPub.destroy();

    this.turtles.delete(request.name);
    WorldState.removeTurtle(request.name);

    this.logInfo(`Killed turtle '${request.name}'`);
    return {};
  }

  /**
   * Handle /clear service
   */
  _handleClear() {
    this.trails = [];
    this._notifyCanvas();
    return {};
  }

  /**
   * Handle /reset service
   */
  _handleReset() {
    // Kill all turtles except turtle1
    for (const [name] of this.turtles) {
      if (name !== 'turtle1') {
        this._handleKill({ name });
      }
    }

    // Reset turtle1
    const turtle1 = this.turtles.get('turtle1');
    if (turtle1) {
      turtle1.x = 5.544445;
      turtle1.y = 5.544445;
      turtle1.theta = 0;
      turtle1.linearVelocity = 0;
      turtle1.angularVelocity = 0;
      WorldState.updateTurtle('turtle1', turtle1.x, turtle1.y, turtle1.theta);
    }

    this.trails = [];
    this._notifyCanvas();
    return {};
  }

  /**
   * Handle cmd_vel message
   */
  _handleCmdVel(turtleName, msg) {
    const turtle = this.turtles.get(turtleName);
    if (!turtle) return;

    turtle.linearVelocity = msg.linear?.x || 0;
    turtle.angularVelocity = msg.angular?.z || 0;
  }

  /**
   * Handle set_pen service
   */
  _handleSetPen(turtleName, request) {
    const turtle = this.turtles.get(turtleName);
    if (!turtle) {
      throw new Error(`Turtle '${turtleName}' not found`);
    }

    turtle.pen = {
      r: request.r,
      g: request.g,
      b: request.b,
      width: request.width,
      off: request.off !== 0
    };

    return {};
  }

  /**
   * Handle teleport_absolute service
   */
  _handleTeleportAbsolute(turtleName, request) {
    const turtle = this.turtles.get(turtleName);
    if (!turtle) {
      throw new Error(`Turtle '${turtleName}' not found`);
    }

    // Teleport without drawing
    turtle.x = request.x;
    turtle.y = request.y;
    turtle.theta = request.theta;
    WorldState.updateTurtle(turtleName, turtle.x, turtle.y, turtle.theta);

    return {};
  }

  /**
   * Handle teleport_relative service
   */
  _handleTeleportRelative(turtleName, request) {
    const turtle = this.turtles.get(turtleName);
    if (!turtle) {
      throw new Error(`Turtle '${turtleName}' not found`);
    }

    // Apply relative movement
    turtle.theta += request.angular;
    turtle.x += Math.cos(turtle.theta) * request.linear;
    turtle.y += Math.sin(turtle.theta) * request.linear;
    WorldState.updateTurtle(turtleName, turtle.x, turtle.y, turtle.theta);

    return {};
  }

  /**
   * Execute rotate_absolute action
   */
  async _executeRotateAbsolute(turtleName, goal, feedbackCb, isCanceled) {
    const turtle = this.turtles.get(turtleName);
    if (!turtle) {
      throw new Error(`Turtle '${turtleName}' not found`);
    }

    // Clear any existing action interval before starting new one
    if (turtle.activeActionInterval) {
      clearInterval(turtle.activeActionInterval);
      turtle.activeActionInterval = null;
    }

    const targetTheta = goal.theta;
    const startTheta = turtle.theta;

    // Calculate shortest rotation
    let delta = targetTheta - turtle.theta;
    while (delta > Math.PI) delta -= 2 * Math.PI;
    while (delta < -Math.PI) delta += 2 * Math.PI;

    const rotationSpeed = 1.0; // rad/s
    const direction = delta > 0 ? 1 : -1;
    const totalDelta = Math.abs(delta);
    let rotated = 0;

    return new Promise((resolve) => {
      const interval = setInterval(() => {
        if (isCanceled()) {
          clearInterval(interval);
          turtle.activeActionInterval = null;
          resolve({ delta: rotated * direction });
          return;
        }

        const step = rotationSpeed / 30; // 30 Hz update
        rotated += step;

        if (rotated >= totalDelta) {
          // Done
          turtle.theta = targetTheta;
          clearInterval(interval);
          turtle.activeActionInterval = null;
          resolve({ delta: totalDelta * direction });
        } else {
          turtle.theta = startTheta + rotated * direction;
          WorldState.updateTurtle(turtleName, turtle.x, turtle.y, turtle.theta);

          // Send feedback
          const remaining = totalDelta - rotated;
          feedbackCb({ remaining });
        }
      }, 1000 / 30);

      // Store interval reference for cleanup
      turtle.activeActionInterval = interval;
    });
  }

  /**
   * Main update loop
   */
  _update() {
    const dt = 1.0 / this.updateRate;
    const now = Date.now();

    for (const [name, turtle] of this.turtles) {
      // Store previous position
      const prevX = turtle.x;
      const prevY = turtle.y;

      // Update position based on velocity
      if (turtle.linearVelocity !== 0 || turtle.angularVelocity !== 0) {
        turtle.theta += turtle.angularVelocity * dt;

        // Normalize theta to [-PI, PI]
        while (turtle.theta > Math.PI) turtle.theta -= 2 * Math.PI;
        while (turtle.theta < -Math.PI) turtle.theta += 2 * Math.PI;

        const newX = turtle.x + Math.cos(turtle.theta) * turtle.linearVelocity * dt;
        const newY = turtle.y + Math.sin(turtle.theta) * turtle.linearVelocity * dt;

        // Clamp to world bounds
        turtle.x = Math.max(0, Math.min(11, newX));
        turtle.y = Math.max(0, Math.min(11, newY));

        WorldState.updateTurtle(name, turtle.x, turtle.y, turtle.theta);

        // Add trail if pen is on and moved
        if (!turtle.pen.off && (turtle.x !== prevX || turtle.y !== prevY)) {
          this.trails.push({
            x1: prevX,
            y1: prevY,
            x2: turtle.x,
            y2: turtle.y,
            color: `rgb(${turtle.pen.r}, ${turtle.pen.g}, ${turtle.pen.b})`,
            width: turtle.pen.width
          });
        }
      }

      // Publish pose at lower rate
      if (now - this.lastPosePublish >= 1000 / this.publishRate) {
        const pose = {
          x: turtle.x,
          y: turtle.y,
          theta: turtle.theta,
          linear_velocity: turtle.linearVelocity,
          angular_velocity: turtle.angularVelocity
        };
        turtle.posePub.publish(pose);
      }

      // Publish lidar scan for turtle1
      if (name === 'turtle1' && this.lidarEnabled && turtle.scanPub) {
        if (now - this.lastLidarPublish >= 1000 / this.lidarPublishRate) {
          const scan = WorldState.lidarScan(turtle.x, turtle.y, turtle.theta, {
            angleMin: -Math.PI,
            angleMax: Math.PI,
            numRays: 360,
            rangeMax: 10.0
          });

          scan.header = {
            stamp: this.now(),
            frame_id: 'base_link'
          };

          turtle.scanPub.publish(scan);
          this.lastLidarPublish = now;
        }
      }
    }

    if (now - this.lastPosePublish >= 1000 / this.publishRate) {
      this.lastPosePublish = now;
    }

    // Notify canvas to redraw
    this._notifyCanvas();
  }

  /**
   * Notify canvas component to redraw
   */
  _notifyCanvas() {
    const event = new CustomEvent('turtlesim-update', {
      detail: {
        turtles: Array.from(this.turtles.values()),
        trails: this.trails,
        background: {
          r: this.getParameter('background_r'),
          g: this.getParameter('background_g'),
          b: this.getParameter('background_b')
        }
      }
    });
    window.dispatchEvent(event);
  }

  /**
   * Get turtle data for canvas
   */
  getTurtleData() {
    return {
      turtles: Array.from(this.turtles.values()),
      trails: this.trails,
      background: {
        r: this.getParameter('background_r'),
        g: this.getParameter('background_g'),
        b: this.getParameter('background_b')
      }
    };
  }
}

// Self-register with the node registry
nodeRegistry.register('turtlesim', 'turtlesim_node', TurtlesimNode);
