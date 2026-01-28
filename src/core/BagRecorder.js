import { SimDDS } from './SimDDS.js';

/**
 * BagRecorder - Records topic messages with timestamps
 */
export class BagRecorder {
  constructor(name, topics = []) {
    this.name = name;
    this.topics = topics;
    this.messages = [];
    this.subscriptions = [];
    this.recording = false;
    this.startTime = null;
    this.endTime = null;
  }

  /**
   * Start recording
   */
  start() {
    if (this.recording) return;

    this.recording = true;
    this.startTime = Date.now();
    this.messages = [];

    // Subscribe to all topics
    for (const topic of this.topics) {
      const topicInfo = SimDDS.getTopicInfo(topic);
      if (topicInfo) {
        const subId = SimDDS.subscribe(topic, topicInfo.type, (msg) => {
          if (this.recording) {
            this.messages.push({
              topic,
              type: topicInfo.type,
              timestamp: Date.now() - this.startTime,
              message: { ...msg }
            });
          }
        });
        this.subscriptions.push(subId);
      }
    }
  }

  /**
   * Stop recording
   */
  stop() {
    if (!this.recording) return;

    this.recording = false;
    this.endTime = Date.now();

    // Unsubscribe from all topics
    for (const subId of this.subscriptions) {
      SimDDS.unsubscribe(subId);
    }
    this.subscriptions = [];
  }

  /**
   * Get recording info
   */
  getInfo() {
    const topicCounts = {};
    for (const msg of this.messages) {
      topicCounts[msg.topic] = (topicCounts[msg.topic] || 0) + 1;
    }

    return {
      name: this.name,
      duration: this.endTime ? (this.endTime - this.startTime) / 1000 : null,
      startTime: this.startTime,
      endTime: this.endTime,
      messageCount: this.messages.length,
      topics: this.topics,
      topicCounts
    };
  }

  /**
   * Get all recorded messages
   */
  getMessages() {
    return this.messages;
  }

  /**
   * Export to JSON
   */
  toJSON() {
    return {
      name: this.name,
      startTime: this.startTime,
      endTime: this.endTime,
      topics: this.topics,
      messages: this.messages
    };
  }

  /**
   * Import from JSON
   */
  static fromJSON(data) {
    const recorder = new BagRecorder(data.name, data.topics);
    recorder.startTime = data.startTime;
    recorder.endTime = data.endTime;
    recorder.messages = data.messages;
    return recorder;
  }

  /**
   * Save to localStorage
   */
  saveToStorage() {
    try {
      const key = `ros2bag_${this.name}`;
      localStorage.setItem(key, JSON.stringify(this.toJSON()));
      return true;
    } catch (e) {
      console.error('Failed to save bag to localStorage:', e);
      return false;
    }
  }

  /**
   * Load from localStorage
   */
  static loadFromStorage(name) {
    try {
      const key = `ros2bag_${name}`;
      const data = localStorage.getItem(key);
      if (data) {
        return BagRecorder.fromJSON(JSON.parse(data));
      }
    } catch (e) {
      console.error('Failed to load bag from localStorage:', e);
    }
    return null;
  }

  /**
   * List all bags in localStorage
   */
  static listStoredBags() {
    const bags = [];
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key.startsWith('ros2bag_')) {
        bags.push(key.substring(8));
      }
    }
    return bags;
  }

  /**
   * Delete a bag from localStorage
   */
  static deleteFromStorage(name) {
    const key = `ros2bag_${name}`;
    localStorage.removeItem(key);
  }
}

/**
 * BagStorage - Manages in-memory bag storage
 */
class BagStorageClass {
  constructor() {
    this.bags = new Map();
  }

  /**
   * Store a bag
   */
  store(recorder) {
    this.bags.set(recorder.name, recorder);
  }

  /**
   * Get a bag by name
   */
  get(name) {
    // Check in-memory first
    if (this.bags.has(name)) {
      return this.bags.get(name);
    }
    // Try localStorage
    return BagRecorder.loadFromStorage(name);
  }

  /**
   * List all bags
   */
  list() {
    const memoryBags = Array.from(this.bags.keys());
    const storageBags = BagRecorder.listStoredBags();
    return [...new Set([...memoryBags, ...storageBags])];
  }

  /**
   * Delete a bag
   */
  delete(name) {
    this.bags.delete(name);
    BagRecorder.deleteFromStorage(name);
  }
}

export const BagStorage = new BagStorageClass();
