import { Node } from '../../core/Node.js';

/**
 * TeleopTwistKeyboardNode - General keyboard teleop using WASD keys
 * Matches teleop_twist_keyboard package behavior
 */
export class TeleopTwistKeyboardNode extends Node {
  constructor(name = 'teleop_twist_keyboard', options = {}) {
    super(name, {
      ...options,
      parameters: {
        speed: 0.5,
        turn: 1.0,
        speed_limit: 2.0,
        turn_limit: 2.0,
        ...options.parameters
      }
    });

    this.targetTopic = options.remappings?.['cmd_vel'] || '/turtle1/cmd_vel';
    this.keyHandler = null;
    this.keyUpHandler = null;
    this.activeKeys = new Set();

    // Current velocity state
    this.currentSpeed = 0.5;
    this.currentTurn = 1.0;
  }

  onInit() {
    // Create publisher
    this.cmdVelPub = this.createPublisher(this.targetTopic, 'geometry_msgs/msg/Twist');

    // Set up keyboard handling
    this.keyHandler = this._handleKeyDown.bind(this);
    this.keyUpHandler = this._handleKeyUp.bind(this);

    window.addEventListener('keydown', this.keyHandler);
    window.addEventListener('keyup', this.keyUpHandler);

    // Publish at regular rate while keys are held
    this.createTimer(100, this._publishVelocity.bind(this));

    this.logInfo('Teleop Twist Keyboard started.');
    this.logInfo('Use WASD keys to move, Q/E to rotate, X to stop.');
    this.logInfo(`Publishing to: ${this.targetTopic}`);

    // Dispatch event to show keyboard hint
    window.dispatchEvent(new CustomEvent('teleop-active', {
      detail: { type: 'wasd-keys', node: this.fullName }
    }));
  }

  onShutdown() {
    if (this.keyHandler) {
      window.removeEventListener('keydown', this.keyHandler);
      window.removeEventListener('keyup', this.keyUpHandler);
    }
    this.activeKeys.clear();

    window.dispatchEvent(new CustomEvent('teleop-inactive', {
      detail: { node: this.fullName }
    }));
  }

  _handleKeyDown(event) {
    const key = event.key.toLowerCase();

    // Movement keys
    if (['w', 'a', 's', 'd', 'q', 'e', 'x'].includes(key)) {
      event.preventDefault();
      this.activeKeys.add(key);
      return;
    }

    // Speed adjustment keys
    if (key === 'i') {
      this.currentSpeed = Math.min(this.getParameter('speed_limit'), this.currentSpeed + 0.1);
      this.logInfo(`Speed: ${this.currentSpeed.toFixed(1)}`);
      event.preventDefault();
      return;
    }
    if (key === 'k') {
      this.currentSpeed = Math.max(0.1, this.currentSpeed - 0.1);
      this.logInfo(`Speed: ${this.currentSpeed.toFixed(1)}`);
      event.preventDefault();
      return;
    }
    if (key === 'o') {
      this.currentTurn = Math.min(this.getParameter('turn_limit'), this.currentTurn + 0.1);
      this.logInfo(`Turn: ${this.currentTurn.toFixed(1)}`);
      event.preventDefault();
      return;
    }
    if (key === 'l') {
      this.currentTurn = Math.max(0.1, this.currentTurn - 0.1);
      this.logInfo(`Turn: ${this.currentTurn.toFixed(1)}`);
      event.preventDefault();
      return;
    }
  }

  _handleKeyUp(event) {
    const key = event.key.toLowerCase();
    if (['w', 'a', 's', 'd', 'q', 'e', 'x'].includes(key)) {
      event.preventDefault();
      this.activeKeys.delete(key);
    }
  }

  _publishVelocity() {
    let linear = 0;
    let angular = 0;

    // Stop command
    if (this.activeKeys.has('x')) {
      // Publish zero velocity
      const twist = {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      };
      this.cmdVelPub.publish(twist);
      this.lastVelocity = false;
      return;
    }

    // Forward/backward
    if (this.activeKeys.has('w')) {
      linear += this.currentSpeed;
    }
    if (this.activeKeys.has('s')) {
      linear -= this.currentSpeed;
    }

    // Strafe (not used in differential drive, but included for completeness)
    // In turtlesim this would be ignored
    if (this.activeKeys.has('a')) {
      angular += this.currentTurn;
    }
    if (this.activeKeys.has('d')) {
      angular -= this.currentTurn;
    }

    // Pure rotation
    if (this.activeKeys.has('q')) {
      angular += this.currentTurn;
    }
    if (this.activeKeys.has('e')) {
      angular -= this.currentTurn;
    }

    // Only publish if there's velocity or we just stopped
    if (linear !== 0 || angular !== 0 || this.lastVelocity) {
      const twist = {
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular }
      };

      this.cmdVelPub.publish(twist);
      this.lastVelocity = linear !== 0 || angular !== 0;
    }
  }
}
