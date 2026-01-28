import { LogManager } from '../core/LogManager.js';

/**
 * RqtConsole - Log viewer with filtering capabilities
 *
 * Accepts optional elements object for dependency injection/testing.
 * Falls back to document.getElementById for backward compatibility.
 */
export class RqtConsole {
  constructor(elements = {}) {
    // Support both explicit elements and ID-based lookup for backward compatibility
    this.panel = elements.panel || document.getElementById('console-panel');
    this.messagesContainer = elements.messages || document.getElementById('console-messages');
    this.filterLevel = elements.filterLevel || document.getElementById('console-filter-level');
    this.filterNode = elements.filterNode || document.getElementById('console-filter-node');
    this.clearButton = elements.clearButton || document.getElementById('console-clear');
    this.pauseButton = elements.pauseButton || document.getElementById('console-pause');
    this.closeButton = elements.closeButton || document.getElementById('console-close');

    this.paused = false;
    this.pendingLogs = [];
    this.maxDisplayedLogs = 500;
    this.unsubscribe = null;

    this._setupEventListeners();
  }

  _setupEventListeners() {
    // Filter level change
    this.filterLevel?.addEventListener('change', () => {
      this._refreshLogs();
    });

    // Filter node change
    this.filterNode?.addEventListener('input', () => {
      this._refreshLogs();
    });

    // Clear button
    this.clearButton?.addEventListener('click', () => {
      this._clearLogs();
    });

    // Pause button
    this.pauseButton?.addEventListener('click', () => {
      this._togglePause();
    });

    // Close button
    this.closeButton?.addEventListener('click', () => {
      this.hide();
    });

    // Subscribe to log events
    this.unsubscribe = LogManager.addListener((entry) => {
      this._handleLogEntry(entry);
    });
  }

  _handleLogEntry(entry) {
    if (this.paused) {
      this.pendingLogs.push(entry);
      return;
    }

    this._addLogEntry(entry);
  }

  _addLogEntry(entry) {
    if (!this.messagesContainer) return;

    // Check filters
    const minLevel = LogManager.parseLevel(this.filterLevel?.value || 'INFO');
    if (entry.levelValue < minLevel) return;

    const nodeFilter = this.filterNode?.value?.toLowerCase();
    if (nodeFilter && !entry.node.toLowerCase().includes(nodeFilter)) return;

    // Create log element
    const logEl = document.createElement('div');
    logEl.className = `log-entry log-${entry.level.toLowerCase()}`;
    logEl.textContent = LogManager.formatEntry(entry);

    // Add to container
    this.messagesContainer.appendChild(logEl);

    // Limit displayed logs
    while (this.messagesContainer.children.length > this.maxDisplayedLogs) {
      this.messagesContainer.removeChild(this.messagesContainer.firstChild);
    }

    // Auto-scroll to bottom
    this.messagesContainer.scrollTop = this.messagesContainer.scrollHeight;
  }

  _refreshLogs() {
    if (!this.messagesContainer) return;

    // Clear current display
    this.messagesContainer.innerHTML = '';

    // Get filtered logs
    const filter = {
      level: this.filterLevel?.value,
      node: this.filterNode?.value
    };

    const logs = LogManager.getLogs(filter);

    // Display last N logs
    const startIdx = Math.max(0, logs.length - this.maxDisplayedLogs);
    for (let i = startIdx; i < logs.length; i++) {
      this._addLogEntry(logs[i]);
    }
  }

  _clearLogs() {
    if (this.messagesContainer) {
      this.messagesContainer.innerHTML = '';
    }
    LogManager.clear();
    this.pendingLogs = [];
  }

  _togglePause() {
    this.paused = !this.paused;

    if (this.pauseButton) {
      this.pauseButton.textContent = this.paused ? 'Resume' : 'Pause';
    }

    if (!this.paused) {
      // Flush pending logs
      for (const entry of this.pendingLogs) {
        this._addLogEntry(entry);
      }
      this.pendingLogs = [];
    }
  }

  /**
   * Show the console
   */
  show() {
    this.panel?.classList.remove('hidden');
    this._refreshLogs();
  }

  /**
   * Hide the console
   */
  hide() {
    this.panel?.classList.add('hidden');
  }

  /**
   * Toggle visibility
   */
  toggle() {
    if (this.panel?.classList.contains('hidden')) {
      this.show();
    } else {
      this.hide();
    }
  }

  /**
   * Destroy the console
   */
  destroy() {
    if (this.unsubscribe) {
      this.unsubscribe();
    }
  }
}
