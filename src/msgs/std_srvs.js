/**
 * std_srvs - Standard ROS 2 service definitions
 */

import { messageRegistry } from './registry.js';

export const Empty = {
  name: 'std_srvs/srv/Empty',
  type: 'service',
  request: {},
  response: {},
  createRequest: () => ({}),
  createResponse: () => ({}),
  definition: `# Empty service - no request or response data.
---`
};

// Self-register all service types
messageRegistry.registerService('std_srvs/srv/Empty', Empty);

// Export all service types
export default { Empty };
