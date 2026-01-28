/**
 * tf2_msgs - TF2 message types
 */

import { TransformStamped } from './geometry_msgs.js';

export const TFMessage = {
  name: 'tf2_msgs/msg/TFMessage',
  fields: {
    transforms: { type: 'geometry_msgs/msg/TransformStamped[]', default: [] }
  },
  create: (data = {}) => ({
    transforms: (data.transforms || []).map(t => TransformStamped.create(t))
  }),
  definition: `# TF2 transform message.
geometry_msgs/TransformStamped[] transforms`
};

// Export all message types
export default {
  TFMessage
};
