/**
 * ROS2 Web Emulator - Main Application Entry
 */

import { TerminalManager } from './ui/TerminalManager.js';
import { Canvas } from './ui/Canvas.js';
import { MapCanvas } from './ui/MapCanvas.js';
import { Graph } from './ui/Graph.js';
import { RqtConsole } from './ui/RqtConsole.js';
import { Layout } from './ui/Layout.js';
import { parseCommand } from './cli/parser.js';
import { ProcessManager } from './core/ProcessManager.js';
import { SimDDS } from './core/SimDDS.js';
import { LogManager } from './core/LogManager.js';

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
    this.logger = LogManager.getLogger('/ros2websim');
  }

  /**
   * Initialize the application
   */
  init() {
    this.logger.info('Initializing ROS2 Web Emulator...');

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
      terminal.writeln('\x1b[32m║           ROS2 Web Emulator - Educational Version            ║\x1b[0m');
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

    this.logger.info('ROS2 Web Emulator initialized');
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

    // Custom events
    window.addEventListener('toggle-graph', () => {
      this.graph.toggle();
    });

    window.addEventListener('toggle-console', () => {
      this.rqtConsole.toggle();
    });

    window.addEventListener('show-map', () => {
      this.layout.showMap();
    });

    window.addEventListener('open-rqt-modal', () => {
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
    window.addEventListener('teleop-active', (event) => {
      const { type } = event.detail;
      this._showTeleopHint(type);
    });

    window.addEventListener('teleop-inactive', () => {
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
    window.addEventListener('canvas-edit-mode-changed', (event) => {
      this._updateObstacleButtons(event.detail.mode);
    });
  }

  _updateObstacleButtons(mode) {
    const addBtn = document.getElementById('add-obstacle-mode');
    const deleteBtn = document.getElementById('delete-obstacle-mode');

    addBtn?.classList.toggle('active', mode === 'add');
    deleteBtn?.classList.toggle('active', mode === 'delete');
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
    const title = document.getElementById('modal-title');
    const body = document.getElementById('modal-body');

    if (!modal || !body) return;

    title.textContent = 'ROS2 Web Emulator - Help';
    body.innerHTML = `
      <div class="help-content">
        <div class="help-section help-intro">
          <p>This emulator lets you learn ROS2 CLI commands in your browser. Follow along with the official tutorial:</p>
          <a href="https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html" target="_blank" class="tutorial-link">
            📚 ROS2 Jazzy - Beginner CLI Tools Tutorial
          </a>
        </div>

        <h3>Quick Start</h3>
        <ol>
          <li>Run <code>ros2 run turtlesim turtlesim_node</code> to start turtlesim</li>
          <li>Open a new terminal with the <strong>+</strong> button</li>
          <li>Run <code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code></li>
          <li>Use <strong>W/A/S/D</strong> keys to move the turtle</li>
        </ol>

        <h3>Available Packages</h3>
        <table class="help-table">
          <tr><td><code>ros2 run turtlesim turtlesim_node</code></td><td>Start turtlesim simulator</td></tr>
          <tr><td><code>ros2 run turtlesim turtle_teleop_key</code></td><td>Control with arrow keys</td></tr>
          <tr><td><code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code></td><td>Control with WASD keys</td></tr>
          <tr><td><code>ros2 run simple_slam slam_node</code></td><td>Start SLAM mapping node</td></tr>
          <tr><td><code>ros2 run tf2_ros static_transform_publisher</code></td><td>Publish static TF</td></tr>
        </table>

        <h3>ros2 run Options</h3>
        <table class="help-table">
          <tr><td><code>--ros-args --remap __node:=name</code></td><td>Rename node</td></tr>
          <tr><td><code>--ros-args --remap topic:=new_topic</code></td><td>Remap topic</td></tr>
          <tr><td><code>--ros-args --log-level WARN</code></td><td>Set log level</td></tr>
        </table>

        <h3>ros2 node Commands</h3>
        <table class="help-table">
          <tr><td><code>ros2 node list</code></td><td>List running nodes</td></tr>
          <tr><td><code>ros2 node info &lt;node&gt;</code></td><td>Show node details</td></tr>
        </table>

        <h3>ros2 topic Commands</h3>
        <table class="help-table">
          <tr><td><code>ros2 topic list</code></td><td>List all topics</td></tr>
          <tr><td><code>ros2 topic list -t</code></td><td>List with types</td></tr>
          <tr><td><code>ros2 topic info &lt;topic&gt;</code></td><td>Show topic info</td></tr>
          <tr><td><code>ros2 topic type &lt;topic&gt;</code></td><td>Show message type</td></tr>
          <tr><td><code>ros2 topic echo &lt;topic&gt;</code></td><td>Print messages</td></tr>
          <tr><td><code>ros2 topic pub &lt;topic&gt; &lt;type&gt; "&lt;yaml&gt;"</code></td><td>Publish message</td></tr>
          <tr><td><code>ros2 topic pub --once ...</code></td><td>Publish once</td></tr>
          <tr><td><code>ros2 topic pub -r 10 ...</code></td><td>Publish at rate</td></tr>
          <tr><td><code>ros2 topic hz &lt;topic&gt;</code></td><td>Show publish rate</td></tr>
        </table>

        <h3>ros2 service Commands</h3>
        <table class="help-table">
          <tr><td><code>ros2 service list</code></td><td>List all services</td></tr>
          <tr><td><code>ros2 service list -t</code></td><td>List with types</td></tr>
          <tr><td><code>ros2 service type &lt;service&gt;</code></td><td>Show service type</td></tr>
          <tr><td><code>ros2 service call &lt;srv&gt; &lt;type&gt; "&lt;yaml&gt;"</code></td><td>Call service</td></tr>
        </table>
        <p><em>Turtlesim services: /spawn, /kill, /turtle1/set_pen, /turtle1/teleport_absolute</em></p>

        <h3>ros2 action Commands</h3>
        <table class="help-table">
          <tr><td><code>ros2 action list</code></td><td>List all actions</td></tr>
          <tr><td><code>ros2 action list -t</code></td><td>List with types</td></tr>
          <tr><td><code>ros2 action info &lt;action&gt;</code></td><td>Show action info</td></tr>
          <tr><td><code>ros2 action send_goal &lt;action&gt; &lt;type&gt; "&lt;yaml&gt;"</code></td><td>Send goal</td></tr>
          <tr><td><code>ros2 action send_goal ... --feedback</code></td><td>With feedback</td></tr>
        </table>
        <p><em>Turtlesim action: /turtle1/rotate_absolute</em></p>

        <h3>ros2 param Commands</h3>
        <table class="help-table">
          <tr><td><code>ros2 param list</code></td><td>List all parameters</td></tr>
          <tr><td><code>ros2 param list &lt;node&gt;</code></td><td>List node params</td></tr>
          <tr><td><code>ros2 param get &lt;node&gt; &lt;param&gt;</code></td><td>Get parameter value</td></tr>
          <tr><td><code>ros2 param set &lt;node&gt; &lt;param&gt; &lt;value&gt;</code></td><td>Set parameter</td></tr>
          <tr><td><code>ros2 param dump &lt;node&gt;</code></td><td>Dump all params</td></tr>
        </table>
        <p><em>Turtlesim params: background_r, background_g, background_b</em></p>

        <h3>ros2 bag Commands</h3>
        <table class="help-table">
          <tr><td><code>ros2 bag record &lt;topics&gt;</code></td><td>Record topics</td></tr>
          <tr><td><code>ros2 bag record -o &lt;name&gt; &lt;topics&gt;</code></td><td>Record with name</td></tr>
          <tr><td><code>ros2 bag record -a</code></td><td>Record all topics</td></tr>
          <tr><td><code>ros2 bag info &lt;bag&gt;</code></td><td>Show bag info</td></tr>
          <tr><td><code>ros2 bag play &lt;bag&gt;</code></td><td>Play back bag</td></tr>
          <tr><td><code>ros2 bag play &lt;bag&gt; --rate 2.0</code></td><td>Play at 2x speed</td></tr>
        </table>

        <h3>ros2 interface Commands</h3>
        <table class="help-table">
          <tr><td><code>ros2 interface show &lt;type&gt;</code></td><td>Show message/service definition</td></tr>
          <tr><td><code>ros2 pkg executables &lt;pkg&gt;</code></td><td>List package executables</td></tr>
        </table>

        <h3>rqt Tools</h3>
        <table class="help-table">
          <tr><td><code>rqt_graph</code></td><td>Node/topic graph visualization</td></tr>
          <tr><td><code>rqt_console</code></td><td>Log message viewer</td></tr>
          <tr><td><code>rqt</code></td><td>Open tool selector</td></tr>
        </table>

        <h3 class="slam-title">🗺️ SLAM Tutorial</h3>
        <div class="slam-tutorial">
          <p>Learn how occupancy grid mapping works with our simple SLAM demo:</p>

          <h4>Step 1: Start Turtlesim</h4>
          <p>In Terminal 1:</p>
          <code>ros2 run turtlesim turtlesim_node</code>
          <p>This spawns a turtle with a simulated LIDAR sensor. The gray boxes are obstacles.</p>

          <h4>Step 2: Start SLAM Node</h4>
          <p>Click <strong>+</strong> to add Terminal 2, then run:</p>
          <code>ros2 run simple_slam slam_node</code>
          <p>The SLAM node subscribes to <code>/scan</code> (LIDAR) and <code>/turtle1/pose</code> (position).</p>

          <h4>Step 3: View the Map</h4>
          <p>Click the <strong>Map</strong> button in the header. You'll see a gray grid (unknown space).</p>

          <h4>Step 4: Start Teleop</h4>
          <p>Click <strong>+</strong> to add Terminal 3, then run:</p>
          <code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code>
          <p>Use <strong>W/A/S/D</strong> keys to drive the turtle around.</p>

          <h4>Step 5: Watch the Map Build</h4>
          <p>As the turtle moves:</p>
          <ul>
            <li><strong>White cells</strong> = Free space (LIDAR rays passed through)</li>
            <li><strong>Black cells</strong> = Obstacles (LIDAR rays hit something)</li>
            <li><strong>Gray cells</strong> = Unknown (not yet scanned)</li>
          </ul>
          <p>Drive around all the obstacles to build a complete map!</p>

          <h4>Inspect SLAM Topics</h4>
          <code>ros2 topic echo /map</code>
          <p>This shows the OccupancyGrid message with cell values (-1=unknown, 0=free, 100=occupied).</p>
        </div>

        <h3>Keyboard Shortcuts</h3>
        <ul>
          <li><strong>Ctrl+C</strong> - Stop running command</li>
          <li><strong>Up/Down</strong> - Command history</li>
        </ul>
      </div>
    `;

    modal.classList.remove('hidden');
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

  // Expose for debugging
  window.ros2app = app;
  window.SimDDS = SimDDS;
  window.ProcessManager = ProcessManager;
});
