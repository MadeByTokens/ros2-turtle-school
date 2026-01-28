import { Terminal } from './Terminal.js';
import { ProcessManager } from '../core/ProcessManager.js';
import { Events } from '../core/Events.js';

/**
 * TerminalManager - Manages multiple terminal panes
 */
export class TerminalManager {
  constructor(options = {}) {
    this.containerId = options.containerId || 'terminals-container';
    this.tabListId = options.tabListId || 'terminal-tabs';
    this.addButtonId = options.addButtonId || 'add-terminal';
    this.maxTerminals = options.maxTerminals || 4;
    this.onCommand = options.onCommand || (() => {});

    this.terminals = new Map();
    this.activeTerminalId = null;
    this.nextTerminalNum = 1;

    this.container = document.getElementById(this.containerId);
    this.tabList = document.querySelector(`#${this.tabListId} .tab-list`);
    this.addButton = document.getElementById(this.addButtonId);

    this._setupEventListeners();
    this._addTerminal(); // Create first terminal
  }

  _setupEventListeners() {
    // Add terminal button
    this.addButton?.addEventListener('click', () => {
      this._addTerminal();
    });

    // Handle window resize
    window.addEventListener('resize', () => {
      this._resizeAll();
    });

    // Handle layout changes
    window.addEventListener(Events.LAYOUT_RESIZE, () => {
      // Delay to allow layout to settle
      setTimeout(() => this._resizeAll(), 100);
    });

    // Listen for process state changes to update tab indicators
    window.addEventListener(Events.PROCESS_STARTED, (event) => {
      const { terminalId } = event.detail || {};
      if (terminalId) this._updateTabIndicator(terminalId, true);
    });

    window.addEventListener(Events.PROCESS_STOPPED, (event) => {
      const { terminalId } = event.detail || {};
      if (terminalId) this._updateTabIndicator(terminalId, false);
    });
  }

  /**
   * Update the tab process indicator dot
   */
  _updateTabIndicator(termId, isRunning) {
    const termData = this.terminals.get(termId);
    if (!termData) return;

    const statusDot = termData.tab.querySelector('.tab-status');
    if (statusDot) {
      statusDot.classList.toggle('running', isRunning);
    }
  }

  _addTerminal() {
    if (this.terminals.size >= this.maxTerminals) {
      return null;
    }

    const termId = `terminal_${this.nextTerminalNum++}`;
    const termNum = this.terminals.size + 1;

    // Create terminal container
    const termContainer = document.createElement('div');
    termContainer.id = termId;
    termContainer.className = 'terminal-pane';
    this.container.appendChild(termContainer);

    // Create tab
    const tab = document.createElement('div');
    tab.className = 'terminal-tab';
    tab.dataset.termId = termId;
    tab.setAttribute('role', 'tab');
    tab.setAttribute('aria-selected', 'false');
    tab.innerHTML = `
      <span class="tab-status" aria-hidden="true"></span>
      <span class="tab-title">Terminal ${termNum}</span>
      <button class="tab-close" aria-label="Close terminal ${termNum}">&times;</button>
    `;
    this.tabList.appendChild(tab);

    // Tab click handler
    tab.addEventListener('click', (e) => {
      if (!e.target.classList.contains('tab-close')) {
        this._activateTerminal(termId);
      }
    });

    // Close button handler
    tab.querySelector('.tab-close').addEventListener('click', (e) => {
      e.stopPropagation();
      this._removeTerminal(termId);
    });

    // Create terminal
    const terminal = new Terminal(termContainer, {
      id: termId,
      onCommand: (cmd, term) => this.onCommand(cmd, term),
      onInterrupt: (term) => this._handleInterrupt(term)
    });

    this.terminals.set(termId, { terminal, tab, container: termContainer });

    // Activate this terminal
    this._activateTerminal(termId);

    // Update add button visibility
    this._updateAddButton();

    return terminal;
  }

  _removeTerminal(termId) {
    const termData = this.terminals.get(termId);
    if (!termData) return;

    // Don't remove the last terminal
    if (this.terminals.size <= 1) {
      return;
    }

    // Kill all processes owned by this terminal
    ProcessManager.killByTerminal(termId);

    // Destroy terminal
    termData.terminal.destroy();
    termData.container.remove();
    termData.tab.remove();

    this.terminals.delete(termId);

    // Activate another terminal if this was active
    if (this.activeTerminalId === termId) {
      const nextTerm = this.terminals.keys().next().value;
      if (nextTerm) {
        this._activateTerminal(nextTerm);
      }
    }

    // Renumber tabs
    this._renumberTabs();

    // Update add button visibility
    this._updateAddButton();
  }

  _activateTerminal(termId) {
    const termData = this.terminals.get(termId);
    if (!termData) return;

    // Deactivate current
    if (this.activeTerminalId && this.activeTerminalId !== termId) {
      const prevData = this.terminals.get(this.activeTerminalId);
      if (prevData) {
        prevData.tab.classList.remove('active');
        prevData.tab.setAttribute('aria-selected', 'false');
        prevData.container.classList.remove('active');
      }
    }

    // Activate new
    termData.tab.classList.add('active');
    termData.tab.setAttribute('aria-selected', 'true');
    termData.container.classList.add('active');
    this.activeTerminalId = termId;

    // Focus and fit
    setTimeout(() => {
      termData.terminal.focus();
      termData.terminal.fit();
    }, 0);
  }

  _handleInterrupt(terminal) {
    // Kill processes owned by this terminal
    ProcessManager.killByTerminal(terminal.id);
  }

  _renumberTabs() {
    let num = 1;
    for (const [, termData] of this.terminals) {
      const title = termData.tab.querySelector('.tab-title');
      if (title) {
        title.textContent = `Terminal ${num}`;
      }
      num++;
    }
  }

  _updateAddButton() {
    if (this.addButton) {
      this.addButton.style.display =
        this.terminals.size >= this.maxTerminals ? 'none' : 'block';
    }
  }

  _resizeAll() {
    for (const [, termData] of this.terminals) {
      termData.terminal.fit();
    }
  }

  /**
   * Get the currently active terminal
   */
  getActiveTerminal() {
    if (!this.activeTerminalId) return null;
    const termData = this.terminals.get(this.activeTerminalId);
    return termData?.terminal || null;
  }

  /**
   * Get a terminal by ID
   */
  getTerminal(termId) {
    return this.terminals.get(termId)?.terminal || null;
  }

  /**
   * Get all terminals
   */
  getAllTerminals() {
    return Array.from(this.terminals.values()).map(td => td.terminal);
  }

  /**
   * Get terminal count
   */
  getTerminalCount() {
    return this.terminals.size;
  }

  /**
   * Clear all terminals
   */
  clearAll() {
    for (const [, termData] of this.terminals) {
      termData.terminal.clear();
      termData.terminal.writePrompt();
    }
  }

  /**
   * Focus the active terminal
   */
  focus() {
    const active = this.getActiveTerminal();
    if (active) {
      active.focus();
    }
  }

  /**
   * Destroy all terminals
   */
  destroy() {
    for (const [termId] of this.terminals) {
      const termData = this.terminals.get(termId);
      if (termData) {
        ProcessManager.killByTerminal(termId);
        termData.terminal.destroy();
      }
    }
    this.terminals.clear();
    this.container.innerHTML = '';
    this.tabList.innerHTML = '';
  }
}
