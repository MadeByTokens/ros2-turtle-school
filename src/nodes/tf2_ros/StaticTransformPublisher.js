import { Node } from '../../core/Node.js';
import { TFBuffer } from '../../core/TFBuffer.js';

/**
 * StaticTransformPublisher - Publishes static transforms
 * Usage: ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
 *    or: ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
 */
export class StaticTransformPublisher extends Node {
  constructor(name = 'static_transform_publisher', options = {}) {
    super(name, options);

    // Parse transform from options or use defaults
    this.transform = {
      translation: { x: 0, y: 0, z: 0 },
      rotation: { x: 0, y: 0, z: 0, w: 1 }
    };
    this.frameId = 'world';
    this.childFrameId = 'base_link';

    // Parse from command line args if provided
    this._parseArgs(options.args || []);
  }

  _parseArgs(args) {
    if (args.length === 0) return;

    // Try to parse arguments
    // Format 1: x y z yaw pitch roll frame_id child_frame_id
    // Format 2: x y z qx qy qz qw frame_id child_frame_id

    if (args.length >= 8) {
      this.transform.translation.x = parseFloat(args[0]) || 0;
      this.transform.translation.y = parseFloat(args[1]) || 0;
      this.transform.translation.z = parseFloat(args[2]) || 0;

      if (args.length === 8) {
        // Euler angles (yaw, pitch, roll)
        const yaw = parseFloat(args[3]) || 0;
        const pitch = parseFloat(args[4]) || 0;
        const roll = parseFloat(args[5]) || 0;
        this.transform.rotation = this._eulerToQuaternion(yaw, pitch, roll);
        this.frameId = args[6];
        this.childFrameId = args[7];
      } else if (args.length >= 9) {
        // Quaternion
        this.transform.rotation.x = parseFloat(args[3]) || 0;
        this.transform.rotation.y = parseFloat(args[4]) || 0;
        this.transform.rotation.z = parseFloat(args[5]) || 0;
        this.transform.rotation.w = parseFloat(args[6]) || 1;
        this.frameId = args[7];
        this.childFrameId = args[8];
      }
    }
  }

  _eulerToQuaternion(yaw, pitch, roll) {
    // Convert Euler angles to quaternion
    const cy = Math.cos(yaw * 0.5);
    const sy = Math.sin(yaw * 0.5);
    const cp = Math.cos(pitch * 0.5);
    const sp = Math.sin(pitch * 0.5);
    const cr = Math.cos(roll * 0.5);
    const sr = Math.sin(roll * 0.5);

    return {
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    };
  }

  onInit() {
    // Create publisher for /tf_static
    this.tfPub = this.createPublisher('/tf_static', 'tf2_msgs/msg/TFMessage');

    // Create the transform message
    const transformStamped = {
      header: {
        stamp: this.now(),
        frame_id: this.frameId
      },
      child_frame_id: this.childFrameId,
      transform: this.transform
    };

    // Add to TF buffer
    TFBuffer.setTransform(transformStamped, true);

    // Publish immediately
    this.tfPub.publish({
      transforms: [transformStamped]
    });

    // Publish periodically (static transforms are latched, but we simulate with periodic publishing)
    this.createTimer(1000, () => {
      const msg = {
        transforms: [{
          header: {
            stamp: this.now(),
            frame_id: this.frameId
          },
          child_frame_id: this.childFrameId,
          transform: this.transform
        }]
      };
      this.tfPub.publish(msg);
    });

    this.logInfo(`Publishing static transform: ${this.frameId} -> ${this.childFrameId}`);
    this.logInfo(`  Translation: (${this.transform.translation.x}, ${this.transform.translation.y}, ${this.transform.translation.z})`);
    this.logInfo(`  Rotation: (${this.transform.rotation.x}, ${this.transform.rotation.y}, ${this.transform.rotation.z}, ${this.transform.rotation.w})`);
  }
}
