/**
 * HelpModal - Tabbed help modal component
 */

export class HelpModal {
  constructor() {
    this.activeTab = 'quickstart';
    this.modal = null;
    this.body = null;
    this.tabContent = {};
  }

  /**
   * Show the help modal
   * @param {HTMLElement} modalElement - The modal overlay element
   */
  show(modalElement) {
    this.modal = modalElement;
    const title = document.getElementById('modal-title');
    this.body = document.getElementById('modal-body');

    if (!this.modal || !this.body) return;

    title.textContent = 'ROS 2 Turtle School - Help';
    this.body.innerHTML = this._renderTabs() + this._renderContent();

    this._setupTabListeners();
    this._showTab(this.activeTab);

    this.modal.classList.remove('hidden');
  }

  _renderTabs() {
    return `
      <div class="help-tabs" role="tablist" aria-label="Help sections">
        <button class="help-tab active" data-tab="quickstart" role="tab" aria-selected="true" aria-controls="panel-quickstart">Quick Start</button>
        <button class="help-tab" data-tab="commands" role="tab" aria-selected="false" aria-controls="panel-commands">Commands</button>
        <button class="help-tab" data-tab="slam" role="tab" aria-selected="false" aria-controls="panel-slam">SLAM Tutorial</button>
      </div>
    `;
  }

  _renderContent() {
    return `
      <div class="help-panels">
        <div id="panel-quickstart" class="help-panel active" role="tabpanel" aria-labelledby="tab-quickstart">
          ${this._renderQuickStart()}
        </div>
        <div id="panel-commands" class="help-panel" role="tabpanel" aria-labelledby="tab-commands" hidden>
          ${this._renderCommands()}
        </div>
        <div id="panel-slam" class="help-panel" role="tabpanel" aria-labelledby="tab-slam" hidden>
          ${this._renderSlam()}
        </div>
      </div>
    `;
  }

  _renderQuickStart() {
    return `
      <div class="help-content">
        <div class="help-section help-intro">
          <p>This emulator lets you learn ROS2 CLI commands in your browser. Follow along with the official tutorial:</p>
          <a href="https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html" target="_blank" rel="noopener" class="tutorial-link">
            ROS2 Jazzy - Beginner CLI Tools Tutorial
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

        <h3>Keyboard Shortcuts</h3>
        <ul>
          <li><strong>Ctrl+C</strong> - Stop running command</li>
          <li><strong>Up/Down</strong> - Command history</li>
        </ul>
      </div>
    `;
  }

  _renderCommands() {
    return `
      <div class="help-content">
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
      </div>
    `;
  }

  _renderSlam() {
    return `
      <div class="help-content">
        <div class="slam-tutorial">
          <p>Learn how occupancy grid mapping works with our simple SLAM demo.</p>

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

          <h4>Step 6: Inspect the System</h4>
          <p>Open a 4th terminal and explore what SLAM is doing:</p>
          <table class="help-table">
            <tr><td><code>ros2 node info /slam_node</code></td><td>See what the SLAM node subscribes to and publishes</td></tr>
            <tr><td><code>ros2 topic list -t</code></td><td>See all active topics with their types</td></tr>
            <tr><td><code>ros2 interface show sensor_msgs/msg/LaserScan</code></td><td>Understand the lidar message structure</td></tr>
            <tr><td><code>ros2 interface show nav_msgs/msg/OccupancyGrid</code></td><td>Understand the map message structure</td></tr>
          </table>

          <h4>Step 7: Monitor Live Data</h4>
          <table class="help-table">
            <tr><td><code>ros2 topic echo /scan</code></td><td>Watch raw lidar data streaming in</td></tr>
            <tr><td><code>ros2 topic echo /map</code></td><td>Watch occupancy grid values update</td></tr>
            <tr><td><code>ros2 topic hz /scan</code></td><td>Check lidar rate (~10 Hz)</td></tr>
            <tr><td><code>ros2 topic hz /map</code></td><td>Check map publish rate (~2 Hz)</td></tr>
            <tr><td><code>ros2 topic bw /scan</code></td><td>Measure lidar bandwidth</td></tr>
          </table>

          <h4>Step 8: Tune Parameters</h4>
          <p>Change SLAM behavior at runtime:</p>
          <table class="help-table">
            <tr><td><code>ros2 param list /slam_node</code></td><td>See all tunable parameters</td></tr>
            <tr><td><code>ros2 param get /slam_node hit_prob</code></td><td>Check obstacle detection confidence</td></tr>
            <tr><td><code>ros2 param set /slam_node hit_prob 0.95</code></td><td>Increase obstacle confidence</td></tr>
            <tr><td><code>ros2 param set /slam_node miss_prob 0.1</code></td><td>More confident free-space marking</td></tr>
            <tr><td><code>ros2 param dump /slam_node</code></td><td>Dump all current parameter values</td></tr>
          </table>

          <h4>Step 9: Record and Replay</h4>
          <p>Record a mapping session for later analysis:</p>
          <table class="help-table">
            <tr><td><code>ros2 bag record -o my_run /scan /turtle1/pose /map</code></td><td>Record SLAM topics</td></tr>
            <tr><td><code>ros2 bag info my_run</code></td><td>Inspect recorded data</td></tr>
            <tr><td><code>ros2 bag play my_run</code></td><td>Replay the session</td></tr>
            <tr><td><code>ros2 bag play my_run --rate 0.5</code></td><td>Replay at half speed</td></tr>
          </table>

          <h4>SLAM Cell Values</h4>
          <p><code>ros2 topic echo /map</code> shows OccupancyGrid data: <strong>-1</strong> = unknown, <strong>0</strong> = free, <strong>100</strong> = occupied.</p>
        </div>
      </div>
    `;
  }

  _setupTabListeners() {
    const tabs = this.body.querySelectorAll('.help-tab');
    tabs.forEach(tab => {
      tab.addEventListener('click', () => {
        const tabName = tab.dataset.tab;
        this._showTab(tabName);
      });
    });
  }

  _showTab(tabName) {
    this.activeTab = tabName;

    // Update tab buttons
    const tabs = this.body.querySelectorAll('.help-tab');
    tabs.forEach(tab => {
      const isActive = tab.dataset.tab === tabName;
      tab.classList.toggle('active', isActive);
      tab.setAttribute('aria-selected', isActive ? 'true' : 'false');
    });

    // Update panels
    const panels = this.body.querySelectorAll('.help-panel');
    panels.forEach(panel => {
      const isActive = panel.id === `panel-${tabName}`;
      panel.classList.toggle('active', isActive);
      panel.hidden = !isActive;
    });
  }
}

// Singleton instance
export const helpModal = new HelpModal();
