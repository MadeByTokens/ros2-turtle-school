import { Node } from '../../core/Node.js';
import { nodeRegistry } from '../registry.js';

/**
 * TurtleTeleopNode - Keyboard teleop for turtlesim using arrow keys
 * Matches the real turtle_teleop_key node behavior
 */
export class TurtleTeleopNode extends Node {
  constructor(name = 'turtle_teleop_key', options = {}) {
    super(name, {
      ...options,
      parameters: {
        scale_linear: 2.0,
        scale_angular: 2.0,
        ...options.parameters
      }
    });

    this.targetTopic = options.remappings?.['cmd_vel'] || '/turtle1/cmd_vel';
    this.keyHandler = null;
    this.activeKeys = new Set();
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

    this.logInfo('Turtle teleop started. Use arrow keys to move the turtle.');
    this.logInfo(`Publishing to: ${this.targetTopic}`);

    // Dispatch event to show keyboard hint
    window.dispatchEvent(new CustomEvent('teleop-active', {
      detail: { type: 'arrow-keys', node: this.fullName }
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
    // Only respond to arrow keys
    if (!['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
      return;
    }

    event.preventDefault();
    this.activeKeys.add(event.key);
  }

  _handleKeyUp(event) {
    if (!['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
      return;
    }

    event.preventDefault();
    this.activeKeys.delete(event.key);
  }

  _publishVelocity() {
    const scaleLinear = this.getParameter('scale_linear');
    const scaleAngular = this.getParameter('scale_angular');

    let linear = 0;
    let angular = 0;

    if (this.activeKeys.has('ArrowUp')) {
      linear += scaleLinear;
    }
    if (this.activeKeys.has('ArrowDown')) {
      linear -= scaleLinear;
    }
    if (this.activeKeys.has('ArrowLeft')) {
      angular += scaleAngular;
    }
    if (this.activeKeys.has('ArrowRight')) {
      angular -= scaleAngular;
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

// Self-register with the node registry
nodeRegistry.register('turtlesim', 'turtle_teleop_key', TurtleTeleopNode);
