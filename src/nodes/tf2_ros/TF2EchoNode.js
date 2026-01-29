import { Node } from '../../core/Node.js';
import { TFBuffer } from '../../core/TFBuffer.js';
import { nodeRegistry } from '../registry.js';

/**
 * TF2EchoNode - Continuously prints transform between two frames
 * Usage: ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
 */
export class TF2EchoNode extends Node {
  constructor(name = 'tf2_echo', options = {}) {
    super(name, options);
    const args = options.args || [];
    this.sourceFrame = args[0] || 'world';
    this.targetFrame = args[1] || 'base_link';
  }

  onInit() {
    this.logInfo(`Listening to transform from ${this.sourceFrame} to ${this.targetFrame}`);
    this.createTimer(1000, this._queryTransform.bind(this));
  }

  _queryTransform() {
    const transform = TFBuffer.lookupTransform(this.targetFrame, this.sourceFrame);
    if (transform) {
      const { translation: t, rotation: r } = transform;
      this.logInfo(`At time ${(Date.now() / 1000).toFixed(3)}`);
      this.logInfo(`- Translation: [${t.x.toFixed(3)}, ${t.y.toFixed(3)}, ${t.z.toFixed(3)}]`);
      this.logInfo(`- Rotation: in Quaternion [${r.x.toFixed(3)}, ${r.y.toFixed(3)}, ${r.z.toFixed(3)}, ${r.w.toFixed(3)}]`);
      // Also show as Euler (yaw only for 2D)
      const yaw = Math.atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z));
      this.logInfo(`- Rotation: in RPY (radian) [0.000, 0.000, ${yaw.toFixed(3)}]`);
    } else {
      this.logWarn(`Could not transform ${this.sourceFrame} to ${this.targetFrame}: frame not found`);
    }
  }
}

nodeRegistry.register('tf2_ros', 'tf2_echo', TF2EchoNode);
