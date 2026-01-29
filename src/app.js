/**
 * ROS 2 Turtle School - Main Application Entry
 */

import { TerminalManager } from './ui/TerminalManager.js';
import { Canvas } from './ui/Canvas.js';
import { MapCanvas } from './ui/MapCanvas.js';
import { Graph } from './ui/Graph.js';
import { RqtConsole } from './ui/RqtConsole.js';
import { Layout } from './ui/Layout.js';
import { helpModal } from './ui/HelpModal.js';
import { parseCommand } from './cli/parser.js';
import { ProcessManager } from './core/ProcessManager.js';
import { SimDDS } from './core/SimDDS.js';
import { LogManager } from './core/LogManager.js';
import { Events } from './core/Events.js';

// Import all nodes (triggers self-registration)
import './nodes/index.js';

// Import xterm CSS
import 'xterm/css/xterm.css';

/**
 * Main application class
 */
class App {
  constructor() {
    this.terminalManager = null;
    this.canvas = null;
    this.mapCanvas = null;
    this.graph = null;
    this.rqtConsole = null;
    this.layout = null;
    this.logger = LogManager.getLogger('/ros2_turtle_school');
  }

  /**
   * Initialize the application
   */
  init() {
    this.logger.info('Initializing ROS 2 Turtle School...');

    // Initialize layout manager
    this.layout = new Layout();

    // Initialize terminal manager
    this.terminalManager = new TerminalManager({
      containerId: 'terminals-container',
      tabListId: 'terminal-tabs',
      addButtonId: 'add-terminal',
      onCommand: (cmd, terminal) => parseCommand(cmd, terminal)
    });

    // Initialize canvas
    this.canvas = new Canvas('turtle-canvas');

    // Initialize map canvas
    this.mapCanvas = new MapCanvas('map-canvas');

    // Initialize graph
    this.graph = new Graph('graph-container');

    // Initialize rqt console
    this.rqtConsole = new RqtConsole();

    // Set up UI event listeners
    this._setupEventListeners();

    // Focus terminal
    this.terminalManager.focus();

    // Show welcome message
    const terminal = this.terminalManager.getActiveTerminal();
    if (terminal) {
      terminal.writeln('');
      terminal.writeln('\x1b[32m╔══════════════════════════════════════════════════════════════╗\x1b[0m');
      terminal.writeln('\x1b[32m║                    ROS 2 Turtle School                       ║\x1b[0m');
      terminal.writeln('\x1b[32m╚══════════════════════════════════════════════════════════════╝\x1b[0m');
      terminal.writeln('');
      terminal.writeln('Welcome! This is a browser-based ROS2 CLI emulator.');
      terminal.writeln('Try these commands to get started:');
      terminal.writeln('');
      terminal.writeln('  \x1b[33mros2 run turtlesim turtlesim_node\x1b[0m  - Start turtlesim');
      terminal.writeln('  \x1b[33mros2 topic list\x1b[0m                   - List topics');
      terminal.writeln('  \x1b[33mros2 node list\x1b[0m                    - List nodes');
      terminal.writeln('  \x1b[33mhelp\x1b[0m                              - Show all commands');
      terminal.writeln('');
      terminal.writeln('Use the \x1b[36m+\x1b[0m button to add more terminal panes.');
      terminal.writeln('Press \x1b[36mCtrl+C\x1b[0m to stop running commands.');
      terminal.writeln('');
      terminal.finishCommand();
    }

    this.logger.info('ROS 2 Turtle School initialized');
  }

  _setupEventListeners() {
    // Toggle graph button
    const graphBtn = document.getElementById('toggle-graph');
    graphBtn?.addEventListener('click', () => {
      this.graph.toggle();
    });

    // Toggle map button
    const mapBtn = document.getElementById('toggle-map');
    mapBtn?.addEventListener('click', () => {
      this.layout.toggleMap();
    });

    // Toggle console button
    const consoleBtn = document.getElementById('toggle-console');
    consoleBtn?.addEventListener('click', () => {
      this.rqtConsole.toggle();
    });

    // Help button
    const helpBtn = document.getElementById('show-help');
    helpBtn?.addEventListener('click', () => {
      this._showHelpModal();
    });

    // Obstacle management buttons
    this._setupObstacleButtons();

    // Map overlay buttons
    this._setupMapOverlayButtons();

    // Costmap toggle
    const costmapBtn = document.getElementById('toggle-costmap');
    costmapBtn?.addEventListener('click', () => {
      const isVisible = this.canvas.toggleCostmap();
      costmapBtn.classList.toggle('active', isVisible);
    });

    // Map quality toggle
    const qualityBtn = document.getElementById('toggle-map-quality');
    qualityBtn?.addEventListener('click', () => {
      const isVisible = this.canvas.toggleMapQuality();
      qualityBtn.classList.toggle('active', isVisible);
    });

    // Custom events
    window.addEventListener(Events.TOGGLE_GRAPH, () => {
      this.graph.toggle();
    });

    window.addEventListener(Events.TOGGLE_CONSOLE, () => {
      this.rqtConsole.toggle();
    });

    window.addEventListener(Events.SHOW_MAP, () => {
      this.layout.showMap();
    });

    window.addEventListener(Events.OPEN_RQT_MODAL, () => {
      this._showRqtModal();
    });

    // Quick command buttons
    const quickButtons = document.querySelectorAll('#quick-commands button');
    quickButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        const cmd = btn.dataset.cmd;
        if (cmd) {
          const terminal = this.terminalManager.getActiveTerminal();
          if (terminal) {
            terminal.writeln(cmd);
            parseCommand(cmd, terminal);
          }
        }
      });
    });

    // Teleop active indicator
    window.addEventListener(Events.TELEOP_ACTIVE, (event) => {
      const { type } = event.detail;
      this._showTeleopHint(type);
    });

    window.addEventListener(Events.TELEOP_INACTIVE, () => {
      this._hideTeleopHint();
    });

    // Handle modal
    const modalOverlay = document.getElementById('modal-overlay');
    const modalClose = document.getElementById('modal-close');

    modalOverlay?.addEventListener('click', (e) => {
      if (e.target === modalOverlay) {
        modalOverlay.classList.add('hidden');
      }
    });

    modalClose?.addEventListener('click', () => {
      modalOverlay?.classList.add('hidden');
    });
  }

  _setupObstacleButtons() {
    const randomizeBtn = document.getElementById('randomize-obstacles');
    const addBtn = document.getElementById('add-obstacle-mode');
    const deleteBtn = document.getElementById('delete-obstacle-mode');
    const clearBtn = document.getElementById('clear-obstacles');
    const resetBtn = document.getElementById('reset-obstacles');

    // Randomize obstacles
    randomizeBtn?.addEventListener('click', () => {
      this.canvas.randomizeObstacles(5);
    });

    // Toggle add mode
    addBtn?.addEventListener('click', () => {
      const currentMode = this.canvas.getEditMode();
      const newMode = currentMode === 'add' ? 'none' : 'add';
      this.canvas.setEditMode(newMode);
      this._updateObstacleButtons(newMode);
    });

    // Toggle delete mode
    deleteBtn?.addEventListener('click', () => {
      const currentMode = this.canvas.getEditMode();
      const newMode = currentMode === 'delete' ? 'none' : 'delete';
      this.canvas.setEditMode(newMode);
      this._updateObstacleButtons(newMode);
    });

    // Clear obstacles
    clearBtn?.addEventListener('click', () => {
      this.canvas.clearObstacles();
    });

    // Reset to defaults
    resetBtn?.addEventListener('click', () => {
      this.canvas.resetObstacles();
    });

    // Listen for edit mode changes
    window.addEventListener(Events.CANVAS_EDIT_MODE_CHANGED, (event) => {
      this._updateObstacleButtons(event.detail.mode);
    });
  }

  _updateObstacleButtons(mode) {
    const addBtn = document.getElementById('add-obstacle-mode');
    const deleteBtn = document.getElementById('delete-obstacle-mode');

    addBtn?.classList.toggle('active', mode === 'add');
    deleteBtn?.classList.toggle('active', mode === 'delete');
  }

  _setupMapOverlayButtons() {
    const toggleBtn = document.getElementById('toggle-map-overlay');
    const opacityControl = document.getElementById('map-opacity-control');
    const opacitySlider = document.getElementById('map-opacity-slider');
    const opacityValue = document.getElementById('map-opacity-value');

    // Toggle map overlay
    toggleBtn?.addEventListener('click', () => {
      const isVisible = this.canvas.toggleMap();
      toggleBtn.classList.toggle('active', isVisible);
      opacityControl?.classList.toggle('hidden', !isVisible);
    });

    // Opacity slider
    opacitySlider?.addEventListener('input', (e) => {
      const opacity = parseInt(e.target.value, 10) / 100;
      this.canvas.setMapOpacity(opacity);
      if (opacityValue) {
        opacityValue.textContent = `${e.target.value}%`;
      }
    });

    // Listen for toggle-map-overlay event (from Layout.toggleMap)
    window.addEventListener(Events.TOGGLE_MAP_OVERLAY, () => {
      const isVisible = this.canvas.toggleMap();
      toggleBtn?.classList.toggle('active', isVisible);
      opacityControl?.classList.toggle('hidden', !isVisible);
    });
  }

  _showRqtModal() {
    const modal = document.getElementById('modal-overlay');
    const title = document.getElementById('modal-title');
    const body = document.getElementById('modal-body');

    if (!modal || !body) return;

    title.textContent = 'rqt - ROS2 Tools';
    body.innerHTML = `
      <div class="rqt-tools">
        <button class="rqt-tool-btn" data-tool="graph">
          <span class="icon">📊</span>
          <span class="label">Node Graph</span>
          <span class="desc">Visualize nodes and topics</span>
        </button>
        <button class="rqt-tool-btn" data-tool="console">
          <span class="icon">📋</span>
          <span class="label">Console</span>
          <span class="desc">View log messages</span>
        </button>
      </div>
    `;

    // Add click handlers
    body.querySelectorAll('.rqt-tool-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const tool = btn.dataset.tool;
        modal.classList.add('hidden');

        if (tool === 'graph') {
          this.graph.toggle();
        } else if (tool === 'console') {
          this.rqtConsole.toggle();
        }
      });
    });

    modal.classList.remove('hidden');
  }

  _showHelpModal() {
    const modal = document.getElementById('modal-overlay');
    helpModal.show(modal);
  }

  _showTeleopHint(type) {
    let hint = document.getElementById('teleop-hint');
    if (!hint) {
      hint = document.createElement('div');
      hint.id = 'teleop-hint';
      document.body.appendChild(hint);
    }

    if (type === 'arrow-keys') {
      hint.innerHTML = 'Use <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> to control turtle';
    } else if (type === 'wasd-keys') {
      hint.innerHTML = 'Use <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> to move, <kbd>Q</kbd><kbd>E</kbd> to rotate';
    }

    hint.classList.add('visible');
  }

  _hideTeleopHint() {
    const hint = document.getElementById('teleop-hint');
    if (hint) {
      hint.classList.remove('visible');
    }
  }

  /**
   * Cleanup and destroy the application
   */
  destroy() {
    ProcessManager.killAll();
    SimDDS.reset();
    this.terminalManager?.destroy();
    this.graph?.destroy();
    this.rqtConsole?.destroy();
  }
}

// Initialize app when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
  const app = new App();
  app.init();

  // Expose for debugging in development only
  if (import.meta.env.DEV) {
    window.ros2app = app;
    window.SimDDS = SimDDS;
    window.ProcessManager = ProcessManager;
  }
});
