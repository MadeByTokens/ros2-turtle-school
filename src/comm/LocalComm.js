import { CommInterface } from './CommInterface.js';
import { normalizeQoS } from '../utils/qos.js';

/**
 * Local in-browser communication implementation.
 * All pub/sub happens within the same browser context.
 */
export class LocalComm extends CommInterface {
  constructor() {
    super();
    this.topics = new Map(); // topic -> { type, publishers: Map, subscribers: Map }
    this.services = new Map(); // serviceName -> { type, handler, nodeId }
    this.actions = new Map(); // actionName -> { type, handlers, nodeId, activeGoals: Map }
    this.nextId = 1;
  }

  _generateId() {
    return `id_${this.nextId++}`;
  }

  _ensureTopic(topic, msgType) {
    if (!this.topics.has(topic)) {
      this.topics.set(topic, {
        type: msgType,
        publishers: new Map(),
        subscribers: new Map()
      });
    } else {
      const existing = this.topics.get(topic);
      if (existing.type !== msgType && msgType !== 'unknown' && existing.type !== 'unknown') {
        console.warn(
          `[WARN] Topic '${topic}' already exists with type '${existing.type}', ` +
          `but was requested with type '${msgType}'. In ROS2, all publishers and ` +
          `subscribers on a topic must use the same type.`
        );
      }
    }
    return this.topics.get(topic);
  }

  subscribe(topic, msgType, callback, nodeId = null, qos) {
    const topicData = this._ensureTopic(topic, msgType);
    const subId = this._generateId();
    topicData.subscribers.set(subId, { callback, msgType, nodeId, qos: normalizeQoS(qos) });
    return subId;
  }

  unsubscribe(subscriptionId) {
    for (const [, topicData] of this.topics) {
      if (topicData.subscribers.has(subscriptionId)) {
        topicData.subscribers.delete(subscriptionId);
        return true;
      }
    }
    return false;
  }

  publish(topic, msgType, message) {
    const topicData = this.topics.get(topic);
    if (!topicData) return;

    // Add timestamp if not present
    const msgWithTimestamp = {
      ...message,
      _timestamp: message._timestamp || Date.now()
    };

    // Deliver to all subscribers
    for (const [, sub] of topicData.subscribers) {
      try {
        sub.callback(msgWithTimestamp);
      } catch (err) {
        console.error(`Error in subscriber callback for ${topic}:`, err);
      }
    }
  }

  advertise(topic, msgType, nodeId, qos) {
    const topicData = this._ensureTopic(topic, msgType);
    const pubId = this._generateId();
    topicData.publishers.set(pubId, { nodeId, msgType, qos: normalizeQoS(qos) });
    return pubId;
  }

  unadvertise(publisherId) {
    for (const [, topicData] of this.topics) {
      if (topicData.publishers.has(publisherId)) {
        topicData.publishers.delete(publisherId);
        return true;
      }
    }
    return false;
  }

  advertiseService(serviceName, srvType, handler, nodeId) {
    const serviceId = this._generateId();
    this.services.set(serviceName, {
      id: serviceId,
      type: srvType,
      handler,
      nodeId
    });
    return serviceId;
  }

  unadvertiseService(serviceId) {
    for (const [name, service] of this.services) {
      if (service.id === serviceId) {
        this.services.delete(name);
        return true;
      }
    }
    return false;
  }

  async callService(serviceName, srvType, request) {
    const service = this.services.get(serviceName);
    if (!service) {
      throw new Error(`Service '${serviceName}' not available`);
    }
    if (service.type !== srvType) {
      throw new Error(`Service type mismatch: expected ${service.type}, got ${srvType}`);
    }
    try {
      const response = await service.handler(request);
      return response;
    } catch (err) {
      throw new Error(`Service call failed: ${err.message}`);
    }
  }

  advertiseAction(actionName, actionType, handlers, nodeId) {
    const actionId = this._generateId();
    this.actions.set(actionName, {
      id: actionId,
      type: actionType,
      handlers,
      nodeId,
      activeGoals: new Map()
    });
    return actionId;
  }

  unadvertiseAction(actionId) {
    for (const [name, action] of this.actions) {
      if (action.id === actionId) {
        // Cancel all active goals
        for (const [goalId, goal] of action.activeGoals) {
          if (goal.cancelCallback) {
            goal.cancelCallback();
          }
        }
        this.actions.delete(name);
        return true;
      }
    }
    return false;
  }

  async sendActionGoal(actionName, actionType, goal, feedbackCallback) {
    const action = this.actions.get(actionName);
    if (!action) {
      throw new Error(`Action '${actionName}' not available`);
    }
    if (action.type !== actionType) {
      throw new Error(`Action type mismatch: expected ${action.type}, got ${actionType}`);
    }

    const goalId = this._generateId();

    // Check if goal is accepted
    if (action.handlers.goalCallback) {
      const accepted = await action.handlers.goalCallback(goal);
      if (!accepted) {
        throw new Error('Goal rejected');
      }
    }

    // Store goal info
    const goalInfo = {
      id: goalId,
      goal,
      feedbackCallback,
      canceled: false
    };
    action.activeGoals.set(goalId, goalInfo);

    // Execute the goal
    try {
      const result = await action.handlers.executeCallback(
        goal,
        goalId,
        (feedback) => {
          if (feedbackCallback && !goalInfo.canceled) {
            feedbackCallback(feedback);
          }
        },
        () => goalInfo.canceled
      );
      action.activeGoals.delete(goalId);
      return result;
    } catch (err) {
      action.activeGoals.delete(goalId);
      throw err;
    }
  }

  cancelActionGoal(actionName, goalId) {
    const action = this.actions.get(actionName);
    if (!action) return false;

    const goalInfo = action.activeGoals.get(goalId);
    if (!goalInfo) return false;

    goalInfo.canceled = true;
    if (action.handlers.cancelCallback) {
      action.handlers.cancelCallback(goalId);
    }
    return true;
  }

  getTopics() {
    const result = [];
    for (const [name, data] of this.topics) {
      result.push({
        name,
        type: data.type,
        publishers: Array.from(data.publishers.values()),
        subscribers: Array.from(data.subscribers.values())
      });
    }
    return result;
  }

  getServices() {
    const result = [];
    for (const [name, data] of this.services) {
      result.push({
        name,
        type: data.type,
        node: data.nodeId
      });
    }
    return result;
  }

  getActions() {
    const result = [];
    for (const [name, data] of this.actions) {
      result.push({
        name,
        type: data.type,
        node: data.nodeId
      });
    }
    return result;
  }

  destroy() {
    // Cancel all active action goals
    for (const [, action] of this.actions) {
      for (const [goalId, goal] of action.activeGoals) {
        goal.canceled = true;
      }
    }

    this.topics.clear();
    this.services.clear();
    this.actions.clear();
  }
}
