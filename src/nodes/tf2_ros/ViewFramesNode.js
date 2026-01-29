import { Node } from '../../core/Node.js';
import { TFBuffer } from '../../core/TFBuffer.js';
import { ProcessManager } from '../../core/ProcessManager.js';
import { nodeRegistry } from '../registry.js';

/**
 * ViewFramesNode - Prints the TF frame tree and exits
 * Usage: ros2 run tf2_ros view_frames
 */
export class ViewFramesNode extends Node {
  constructor(name = 'view_frames', options = {}) {
    super(name, options);
  }

  onInit() {
    const tree = TFBuffer.getFrameTree();
    const frames = Object.keys(tree);

    if (frames.length === 0) {
      this.logInfo('No transform data available. Publish transforms first.');
      this.logInfo('Example: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world base_link');
    } else {
      // Find root frames (frames with no parents)
      const roots = frames.filter(f => tree[f].parents.length === 0);

      this.logInfo(`Frames (${frames.length}):`);
      if (roots.length === 0) {
        // No clear root - just list all frames
        for (const frame of frames.sort()) {
          const parents = tree[frame].parents.join(', ');
          const children = tree[frame].children.join(', ');
          this.logInfo(`  ${frame}  parents: [${parents}]  children: [${children}]`);
        }
      } else {
        for (const root of roots) {
          this._printTree(root, tree, '', true);
        }
      }
    }

    // Self-destruct after output
    setTimeout(() => {
      const proc = ProcessManager.getProcessByNodeName(this.fullName);
      if (proc) {
        ProcessManager.kill(proc.processId);
      }
    }, 100);
  }

  _printTree(frame, tree, prefix, isLast) {
    const connector = prefix === '' ? '' : (isLast ? '\\-- ' : '+-- ');
    this.logInfo(prefix + connector + frame);

    const children = tree[frame].children;
    const newPrefix = prefix === '' ? '    ' : prefix + (isLast ? '    ' : '|   ');

    children.forEach((child, index) => {
      const isLastChild = index === children.length - 1;
      this._printTree(child, tree, newPrefix, isLastChild);
    });
  }
}

nodeRegistry.register('tf2_ros', 'view_frames', ViewFramesNode);
