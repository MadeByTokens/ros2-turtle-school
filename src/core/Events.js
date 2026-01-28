/**
 * Events - Centralized event name constants for window.dispatchEvent/addEventListener
 * Using constants prevents typos and enables IDE autocompletion.
 */

export const Events = {
  // Turtlesim canvas updates
  TURTLESIM_UPDATE: 'turtlesim-update',

  // Sensor data updates
  LIDAR_UPDATE: 'lidar-update',

  // Map/SLAM updates
  MAP_UPDATE: 'map-update',
  ROBOT_POSE_UPDATE: 'robot-pose-update',
  SHOW_MAP: 'show-map',
  TOGGLE_MAP_OVERLAY: 'toggle-map-overlay',

  // Process lifecycle
  PROCESS_STARTED: 'process-started',
  PROCESS_STOPPED: 'process-stopped',

  // World state
  WORLD_STATE_CHANGED: 'world-state-changed',

  // Layout events
  LAYOUT_RESIZE: 'layout-resize',

  // Teleop state
  TELEOP_ACTIVE: 'teleop-active',
  TELEOP_INACTIVE: 'teleop-inactive',

  // UI panel toggles
  TOGGLE_GRAPH: 'toggle-graph',
  TOGGLE_CONSOLE: 'toggle-console',

  // Canvas edit mode
  CANVAS_EDIT_MODE_CHANGED: 'canvas-edit-mode-changed',

  // Modals
  OPEN_RQT_MODAL: 'open-rqt-modal'
};
