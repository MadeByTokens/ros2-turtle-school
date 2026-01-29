import { Node } from '../../core/Node.js';
import { TFBuffer } from '../../core/TFBuffer.js';
import { nodeRegistry } from '../registry.js';

/**
 * TF2MonitorNode - Monitors all active TF broadcasts and shows rates
 * Usage: ros2 run tf2_ros tf2_monitor
 */
export class TF2MonitorNode extends Node {
  constructor(name = 'tf2_monitor', options = {}) {
    super(name, options);
    this.frameCounts = new Map();  // "parent->child" -> count
    this.lastResetTime = Date.now();
  }

  onInit() {
    this.createSubscription('/tf', 'tf2_msgs/msg/TFMessage', this._handleTF.bind(this));
    this.createSubscription('/tf_static', 'tf2_msgs/msg/TFMessage', this._handleStaticTF.bind(this));
    this.createTimer(1000, this._displayStatus.bind(this));
    this.logInfo('Monitoring all transforms. Press Ctrl+C to stop.');
  }

  _handleTF(msg) {
    if (!msg.transforms) return;
    for (const tf of msg.transforms) {
      const key = `${tf.header.frame_id} -> ${tf.child_frame_id}`;
      this.frameCounts.set(key, (this.frameCounts.get(key) || 0) + 1);
    }
  }

  _handleStaticTF(msg) {
    if (!msg.transforms) return;
    for (const tf of msg.transforms) {
      const key = `${tf.header.frame_id} -> ${tf.child_frame_id} [static]`;
      this.frameCounts.set(key, (this.frameCounts.get(key) || 0) + 1);
    }
  }

  _displayStatus() {
    const now = Date.now();
    const elapsed = (now - this.lastResetTime) / 1000;
    const tree = TFBuffer.getFrameTree();
    const frames = Object.keys(tree);

    this.logInfo('');
    this.logInfo(`Frames: ${frames.length}`);

    if (this.frameCounts.size === 0) {
      this.logInfo('No transforms received yet.');
    } else {
      for (const [key, count] of this.frameCounts.entries()) {
        const rate = elapsed > 0 ? (count / elapsed).toFixed(1) : '0.0';
        this.logInfo(`  ${key}  Average rate: ${rate} Hz`);
      }
    }

    // Reset counters periodically
    this.frameCounts.clear();
    this.lastResetTime = now;
  }
}

nodeRegistry.register('tf2_ros', 'tf2_monitor', TF2MonitorNode);
