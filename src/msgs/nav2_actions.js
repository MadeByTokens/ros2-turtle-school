/**
 * nav2_msgs - Nav2 action definitions for path planning
 */

import { messageRegistry } from './registry.js';

export const NavigateToPose = {
  name: 'nav2_msgs/action/NavigateToPose',
  type: 'action',
  goal: {
    pose: { type: 'geometry_msgs/msg/PoseStamped' }
  },
  result: {},
  feedback: {
    current_pose: { type: 'geometry_msgs/msg/PoseStamped' },
    distance_remaining: { type: 'float32', default: 0.0 }
  },
  createGoal: (data = {}) => ({
    pose: {
      header: data.pose?.header || { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
      pose: {
        position: {
          x: data.pose?.pose?.position?.x ?? data.pose?.position?.x ?? 0.0,
          y: data.pose?.pose?.position?.y ?? data.pose?.position?.y ?? 0.0,
          z: data.pose?.pose?.position?.z ?? data.pose?.position?.z ?? 0.0
        },
        orientation: { x: 0, y: 0, z: 0, w: 1 }
      }
    }
  }),
  createResult: () => ({}),
  createFeedback: (data = {}) => ({
    current_pose: {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
      pose: {
        position: { x: data.current_pose?.pose?.position?.x ?? 0, y: data.current_pose?.pose?.position?.y ?? 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 }
      }
    },
    distance_remaining: data.distance_remaining ?? 0.0
  }),
  definition: `# Navigate to a pose.
# Goal
geometry_msgs/PoseStamped pose
---
# Result (empty - status conveyed by action state)
---
# Feedback
geometry_msgs/PoseStamped current_pose
float32 distance_remaining`
};

messageRegistry.registerAction('nav2_msgs/action/NavigateToPose', NavigateToPose);

export default { NavigateToPose };
