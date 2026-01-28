/**
 * TFBuffer - Transform buffer for TF2 functionality
 * Stores and retrieves transforms between coordinate frames
 */
class TFBufferClass {
  constructor() {
    // transforms[childFrame][parentFrame] = transform
    this.transforms = new Map();
    this.staticTransforms = new Map();
    this.bufferDuration = 10000; // 10 seconds buffer
  }

  /**
   * Set a transform
   * @param {Object} transform - TransformStamped message
   * @param {boolean} isStatic - Whether this is a static transform
   */
  setTransform(transform, isStatic = false) {
    const { header, child_frame_id, transform: tf } = transform;
    const parentFrame = header.frame_id;
    const childFrame = child_frame_id;

    const storage = isStatic ? this.staticTransforms : this.transforms;

    if (!storage.has(childFrame)) {
      storage.set(childFrame, new Map());
    }

    const frameTransforms = storage.get(childFrame);

    if (!frameTransforms.has(parentFrame)) {
      frameTransforms.set(parentFrame, []);
    }

    const tfList = frameTransforms.get(parentFrame);
    const entry = {
      timestamp: header.stamp.sec * 1000 + header.stamp.nanosec / 1000000,
      transform: tf
    };

    if (isStatic) {
      // Static transforms just keep the latest
      frameTransforms.set(parentFrame, [entry]);
    } else {
      tfList.push(entry);
      // Cleanup old transforms
      this._cleanupBuffer(tfList);
    }
  }

  /**
   * Cleanup old transforms from buffer
   */
  _cleanupBuffer(tfList) {
    const now = Date.now();
    const cutoff = now - this.bufferDuration;

    while (tfList.length > 0 && tfList[0].timestamp < cutoff) {
      tfList.shift();
    }
  }

  /**
   * Lookup transform from source to target frame
   * @param {string} targetFrame - Target frame ID
   * @param {string} sourceFrame - Source frame ID
   * @param {number} time - Time to lookup (ms), 0 for latest
   * @returns {Object|null} Transform or null if not found
   */
  lookupTransform(targetFrame, sourceFrame, time = 0) {
    // Direct transform
    let transform = this._getDirectTransform(sourceFrame, targetFrame, time);
    if (transform) {
      return transform;
    }

    // Inverse transform
    transform = this._getDirectTransform(targetFrame, sourceFrame, time);
    if (transform) {
      return this._invertTransform(transform);
    }

    // Try to find path through tree
    const path = this._findPath(sourceFrame, targetFrame);
    if (path) {
      return this._chainTransforms(path, time);
    }

    return null;
  }

  /**
   * Get direct transform between frames
   */
  _getDirectTransform(childFrame, parentFrame, time) {
    // Check static transforms first
    const staticFrames = this.staticTransforms.get(childFrame);
    if (staticFrames && staticFrames.has(parentFrame)) {
      const entries = staticFrames.get(parentFrame);
      if (entries.length > 0) {
        return entries[entries.length - 1].transform;
      }
    }

    // Check dynamic transforms
    const dynamicFrames = this.transforms.get(childFrame);
    if (dynamicFrames && dynamicFrames.has(parentFrame)) {
      const entries = dynamicFrames.get(parentFrame);
      if (entries.length === 0) return null;

      if (time === 0) {
        return entries[entries.length - 1].transform;
      }

      // Find closest transform to requested time
      for (let i = entries.length - 1; i >= 0; i--) {
        if (entries[i].timestamp <= time) {
          return entries[i].transform;
        }
      }
      return entries[0].transform;
    }

    return null;
  }

  /**
   * Invert a transform
   */
  _invertTransform(transform) {
    // Invert translation and rotation
    const { translation, rotation } = transform;

    // For quaternion, invert by negating x, y, z (conjugate for unit quaternion)
    const invRotation = {
      x: -rotation.x,
      y: -rotation.y,
      z: -rotation.z,
      w: rotation.w
    };

    // Rotate translation by inverse rotation and negate
    const invTranslation = this._rotateVector(
      { x: -translation.x, y: -translation.y, z: -translation.z },
      invRotation
    );

    return {
      translation: invTranslation,
      rotation: invRotation
    };
  }

  /**
   * Rotate a vector by a quaternion
   */
  _rotateVector(v, q) {
    // Quaternion rotation: q * v * q^-1
    const { x, y, z, w } = q;

    // Simplified rotation formula
    const t = [
      2 * (y * v.z - z * v.y),
      2 * (z * v.x - x * v.z),
      2 * (x * v.y - y * v.x)
    ];

    return {
      x: v.x + w * t[0] + y * t[2] - z * t[1],
      y: v.y + w * t[1] + z * t[0] - x * t[2],
      z: v.z + w * t[2] + x * t[1] - y * t[0]
    };
  }

  /**
   * Find path between frames using BFS
   */
  _findPath(sourceFrame, targetFrame) {
    const allFrames = this._getAllFrames();
    const visited = new Set();
    const queue = [[sourceFrame, [sourceFrame]]];

    while (queue.length > 0) {
      const [current, path] = queue.shift();

      if (current === targetFrame) {
        return path;
      }

      if (visited.has(current)) continue;
      visited.add(current);

      // Find connected frames
      const connected = this._getConnectedFrames(current);
      for (const frame of connected) {
        if (!visited.has(frame)) {
          queue.push([frame, [...path, frame]]);
        }
      }
    }

    return null;
  }

  /**
   * Get all frames connected to a given frame
   */
  _getConnectedFrames(frame) {
    const connected = new Set();

    // Frames where this is the child
    for (const [storage] of [[this.transforms], [this.staticTransforms]]) {
      const frameData = storage.get(frame);
      if (frameData) {
        for (const parent of frameData.keys()) {
          connected.add(parent);
        }
      }

      // Frames where this is the parent
      for (const [child, parents] of storage) {
        if (parents.has(frame)) {
          connected.add(child);
        }
      }
    }

    return connected;
  }

  /**
   * Chain transforms along a path
   */
  _chainTransforms(path, time) {
    let result = {
      translation: { x: 0, y: 0, z: 0 },
      rotation: { x: 0, y: 0, z: 0, w: 1 }
    };

    for (let i = 0; i < path.length - 1; i++) {
      const from = path[i];
      const to = path[i + 1];

      let tf = this._getDirectTransform(from, to, time);
      if (!tf) {
        tf = this._getDirectTransform(to, from, time);
        if (tf) {
          tf = this._invertTransform(tf);
        } else {
          return null;
        }
      }

      result = this._composeTransforms(result, tf);
    }

    return result;
  }

  /**
   * Compose two transforms
   */
  _composeTransforms(t1, t2) {
    // Compose rotations (quaternion multiplication)
    const r1 = t1.rotation;
    const r2 = t2.rotation;
    const rotation = {
      x: r1.w * r2.x + r1.x * r2.w + r1.y * r2.z - r1.z * r2.y,
      y: r1.w * r2.y - r1.x * r2.z + r1.y * r2.w + r1.z * r2.x,
      z: r1.w * r2.z + r1.x * r2.y - r1.y * r2.x + r1.z * r2.w,
      w: r1.w * r2.w - r1.x * r2.x - r1.y * r2.y - r1.z * r2.z
    };

    // Compose translations
    const rotatedT2 = this._rotateVector(t2.translation, r1);
    const translation = {
      x: t1.translation.x + rotatedT2.x,
      y: t1.translation.y + rotatedT2.y,
      z: t1.translation.z + rotatedT2.z
    };

    return { translation, rotation };
  }

  /**
   * Get all known frames
   */
  _getAllFrames() {
    const frames = new Set();

    for (const storage of [this.transforms, this.staticTransforms]) {
      for (const [child, parents] of storage) {
        frames.add(child);
        for (const parent of parents.keys()) {
          frames.add(parent);
        }
      }
    }

    return frames;
  }

  /**
   * Get frame tree as adjacency list
   */
  getFrameTree() {
    const tree = {};
    const frames = this._getAllFrames();

    for (const frame of frames) {
      tree[frame] = {
        parents: [],
        children: []
      };
    }

    for (const storage of [this.transforms, this.staticTransforms]) {
      for (const [child, parents] of storage) {
        for (const parent of parents.keys()) {
          if (tree[child]) tree[child].parents.push(parent);
          if (tree[parent]) tree[parent].children.push(child);
        }
      }
    }

    return tree;
  }

  /**
   * Clear all transforms
   */
  clear() {
    this.transforms.clear();
    this.staticTransforms.clear();
  }
}

// Singleton instance
export const TFBuffer = new TFBufferClass();
