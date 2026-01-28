/**
 * EventBus - Application-level event bus singleton.
 * Provides a centralized way to emit and subscribe to events
 * without using window.dispatchEvent/addEventListener.
 *
 * Supported events:
 * - node:started        { nodeId, nodeName }
 * - node:stopped        { nodeId, nodeName }
 * - command:executed    { command, terminal }
 * - canvas:updated      { turtles, trails, background }
 * - map:updated         { map, info }
 * - lidar:updated       { x, y, theta, scan }
 * - teleop:active       { type, node }
 * - teleop:inactive     { node }
 * - graph:toggle        {}
 * - console:toggle      {}
 * - modal:open          { type, data }
 */

class EventBusClass {
  constructor() {
    this.listeners = new Map();
  }

  /**
   * Subscribe to an event
   * @param {string} event - Event name
   * @param {Function} callback - Callback function
   * @returns {Function} Unsubscribe function
   */
  on(event, callback) {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set());
    }
    this.listeners.get(event).add(callback);

    // Return unsubscribe function
    return () => this.off(event, callback);
  }

  /**
   * Subscribe to an event (one-time)
   * @param {string} event - Event name
   * @param {Function} callback - Callback function
   */
  once(event, callback) {
    const wrapper = (data) => {
      this.off(event, wrapper);
      callback(data);
    };
    this.on(event, wrapper);
  }

  /**
   * Unsubscribe from an event
   * @param {string} event - Event name
   * @param {Function} callback - Callback function
   */
  off(event, callback) {
    const eventListeners = this.listeners.get(event);
    if (eventListeners) {
      eventListeners.delete(callback);
    }
  }

  /**
   * Emit an event
   * @param {string} event - Event name
   * @param {*} data - Event data
   */
  emit(event, data = {}) {
    const eventListeners = this.listeners.get(event);
    if (eventListeners) {
      for (const callback of eventListeners) {
        try {
          callback(data);
        } catch (err) {
          console.error(`Error in event listener for '${event}':`, err);
        }
      }
    }

    // Also emit to window for backward compatibility with existing code
    window.dispatchEvent(new CustomEvent(event, { detail: data }));
  }

  /**
   * Remove all listeners for an event
   * @param {string} event - Event name (optional, if not provided removes all)
   */
  clear(event) {
    if (event) {
      this.listeners.delete(event);
    } else {
      this.listeners.clear();
    }
  }

  /**
   * Get the number of listeners for an event
   * @param {string} event - Event name
   * @returns {number}
   */
  listenerCount(event) {
    const eventListeners = this.listeners.get(event);
    return eventListeners ? eventListeners.size : 0;
  }
}

export const EventBus = new EventBusClass();
