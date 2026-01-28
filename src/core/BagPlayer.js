import { SimDDS } from './SimDDS.js';

/**
 * BagPlayer - Plays back recorded bag messages
 */
export class BagPlayer {
  constructor(recorder) {
    this.recorder = recorder;
    this.messages = recorder.getMessages();
    this.playing = false;
    this.paused = false;
    this.rate = 1.0;
    this.currentIndex = 0;
    this.playStartTime = null;
    this.pauseTime = null;
    this.timeouts = [];
    this.onComplete = null;
    this.onMessage = null;
  }

  /**
   * Start playback
   * @param {number} rate - Playback rate (1.0 = normal, 2.0 = 2x speed)
   */
  play(rate = 1.0) {
    if (this.playing) return;

    this.rate = rate;
    this.playing = true;
    this.paused = false;
    this.currentIndex = 0;
    this.playStartTime = Date.now();

    this._scheduleMessages();
  }

  /**
   * Schedule all messages for playback
   */
  _scheduleMessages() {
    // Clear any existing timeouts
    this._clearTimeouts();

    for (let i = this.currentIndex; i < this.messages.length; i++) {
      const msg = this.messages[i];
      const delay = msg.timestamp / this.rate;

      const timeout = setTimeout(() => {
        if (this.playing && !this.paused) {
          this._publishMessage(msg);
          this.currentIndex = i + 1;

          // Check if playback complete
          if (this.currentIndex >= this.messages.length) {
            this.stop();
            if (this.onComplete) {
              this.onComplete();
            }
          }
        }
      }, delay);

      this.timeouts.push(timeout);
    }
  }

  /**
   * Publish a single message
   */
  _publishMessage(msg) {
    SimDDS.publish(msg.topic, msg.type, msg.message);
    if (this.onMessage) {
      this.onMessage(msg);
    }
  }

  /**
   * Clear all scheduled timeouts
   */
  _clearTimeouts() {
    for (const timeout of this.timeouts) {
      clearTimeout(timeout);
    }
    this.timeouts = [];
  }

  /**
   * Pause playback
   */
  pause() {
    if (!this.playing || this.paused) return;

    this.paused = true;
    this.pauseTime = Date.now();
    this._clearTimeouts();
  }

  /**
   * Resume playback
   */
  resume() {
    if (!this.playing || !this.paused) return;

    this.paused = false;
    const pauseDuration = Date.now() - this.pauseTime;
    this.playStartTime += pauseDuration;

    // Reschedule remaining messages
    this._scheduleRemainingMessages();
  }

  /**
   * Schedule remaining messages after resume
   */
  _scheduleRemainingMessages() {
    const elapsed = (Date.now() - this.playStartTime) * this.rate;

    for (let i = this.currentIndex; i < this.messages.length; i++) {
      const msg = this.messages[i];
      const delay = Math.max(0, (msg.timestamp - elapsed) / this.rate);

      const timeout = setTimeout(() => {
        if (this.playing && !this.paused) {
          this._publishMessage(msg);
          this.currentIndex = i + 1;

          if (this.currentIndex >= this.messages.length) {
            this.stop();
            if (this.onComplete) {
              this.onComplete();
            }
          }
        }
      }, delay);

      this.timeouts.push(timeout);
    }
  }

  /**
   * Stop playback
   */
  stop() {
    this.playing = false;
    this.paused = false;
    this._clearTimeouts();
  }

  /**
   * Change playback rate
   */
  setRate(rate) {
    if (this.playing && !this.paused) {
      // Recalculate timing for remaining messages
      const wasPlaying = true;
      this.pause();
      this.rate = rate;
      if (wasPlaying) {
        this.resume();
      }
    } else {
      this.rate = rate;
    }
  }

  /**
   * Get playback status
   */
  getStatus() {
    return {
      playing: this.playing,
      paused: this.paused,
      rate: this.rate,
      progress: this.currentIndex / this.messages.length,
      currentIndex: this.currentIndex,
      totalMessages: this.messages.length
    };
  }
}
