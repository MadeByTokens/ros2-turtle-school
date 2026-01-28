/**
 * turtlesim_actions - Turtlesim action definitions
 */

import { messageRegistry } from './registry.js';

export const RotateAbsolute = {
  name: 'turtlesim/action/RotateAbsolute',
  type: 'action',
  goal: {
    theta: { type: 'float32', default: 0.0 }
  },
  result: {
    delta: { type: 'float32' }
  },
  feedback: {
    remaining: { type: 'float32' }
  },
  createGoal: (data = {}) => ({
    theta: data.theta ?? 0.0
  }),
  createResult: (data = {}) => ({
    delta: data.delta ?? 0.0
  }),
  createFeedback: (data = {}) => ({
    remaining: data.remaining ?? 0.0
  }),
  definition: `# Rotate the turtle to an absolute orientation.
# Goal
float32 theta
---
# Result
float32 delta
---
# Feedback
float32 remaining`
};

// Self-register all action types
messageRegistry.registerAction('turtlesim/action/RotateAbsolute', RotateAbsolute);

// Export all action types
export default {
  RotateAbsolute
};
