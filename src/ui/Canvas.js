import { WorldState } from '../core/WorldState.js';

/**
 * Canvas - 2D visualization for turtle and obstacles
 */
export class Canvas {
  constructor(canvasId = 'turtle-canvas') {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');

    // World dimensions (turtlesim uses 11x11)
    this.worldWidth = 11;
    this.worldHeight = 11;

    // Turtle image
    this.turtleImage = new Image();
    this.turtleImageLoaded = false;
    this._loadTurtleImage();

    // Visualization options
    this.showObstacles = true;
    this.showLidar = false;
    this.showTF = false;

    // Edit mode: 'none', 'add', 'delete'
    this.editMode = 'none';
    this.newObstacleSize = 1.0;

    // Current state
    this.turtles = [];
    this.trails = [];
    this.background = { r: 69, g: 86, b: 255 };
    this.lidarData = null;

    // Set up event listeners
    this._setupEventListeners();
    this._setupClickHandlers();

    // Initial render
    this._render();
  }

  _loadTurtleImage() {
    // Create a simple turtle shape as data URL
    const turtleSvg = `
      <svg xmlns="http://www.w3.org/2000/svg" width="48" height="48" viewBox="0 0 48 48">
        <!-- Shell -->
        <ellipse cx="24" cy="24" rx="16" ry="14" fill="#228B22" stroke="#006400" stroke-width="2"/>
        <!-- Shell pattern -->
        <path d="M24 10 L24 38" stroke="#006400" stroke-width="1"/>
        <path d="M12 24 L36 24" stroke="#006400" stroke-width="1"/>
        <path d="M16 14 L32 34" stroke="#006400" stroke-width="1"/>
        <path d="M32 14 L16 34" stroke="#006400" stroke-width="1"/>
        <!-- Head -->
        <circle cx="40" cy="24" r="6" fill="#90EE90" stroke="#228B22" stroke-width="1.5"/>
        <!-- Eyes -->
        <circle cx="42" cy="22" r="1.5" fill="#000"/>
        <circle cx="42" cy="26" r="1.5" fill="#000"/>
        <!-- Legs -->
        <ellipse cx="14" cy="12" rx="5" ry="4" fill="#90EE90" stroke="#228B22" stroke-width="1"/>
        <ellipse cx="14" cy="36" rx="5" ry="4" fill="#90EE90" stroke="#228B22" stroke-width="1"/>
        <ellipse cx="30" cy="12" rx="5" ry="4" fill="#90EE90" stroke="#228B22" stroke-width="1"/>
        <ellipse cx="30" cy="36" rx="5" ry="4" fill="#90EE90" stroke="#228B22" stroke-width="1"/>
        <!-- Tail -->
        <path d="M6 24 Q2 24 4 22" stroke="#90EE90" stroke-width="3" fill="none"/>
      </svg>
    `;

    const blob = new Blob([turtleSvg], { type: 'image/svg+xml' });
    const url = URL.createObjectURL(blob);

    this.turtleImage.onload = () => {
      this.turtleImageLoaded = true;
      URL.revokeObjectURL(url);
      this._render();
    };

    this.turtleImage.src = url;
  }

  _setupEventListeners() {
    // Listen for turtlesim updates
    window.addEventListener('turtlesim-update', (event) => {
      const { turtles, trails, background } = event.detail;
      this.turtles = turtles;
      this.trails = trails;
      this.background = background;
      this._render();
    });

    // Listen for lidar data
    window.addEventListener('lidar-update', (event) => {
      this.lidarData = event.detail;
      if (this.showLidar) {
        this._render();
      }
    });

    // Handle resize
    window.addEventListener('resize', () => this._handleResize());
    window.addEventListener('layout-resize', () => this._handleResize());

    // Listen for world state changes
    window.addEventListener('world-state-changed', () => this._render());

    // Initial size
    this._handleResize();
  }

  _setupClickHandlers() {
    this.canvas.addEventListener('click', (event) => {
      if (this.editMode === 'none') return;

      const rect = this.canvas.getBoundingClientRect();
      const canvasX = event.clientX - rect.left;
      const canvasY = event.clientY - rect.top;

      // Convert to world coordinates
      const worldCoords = this._canvasToWorld(canvasX, canvasY);

      if (this.editMode === 'add') {
        WorldState.addObstacleAt(worldCoords.x, worldCoords.y, this.newObstacleSize, this.newObstacleSize);
      } else if (this.editMode === 'delete') {
        WorldState.removeObstacleAt(worldCoords.x, worldCoords.y);
      }
    });

    // Show cursor hint based on edit mode
    this.canvas.addEventListener('mousemove', (event) => {
      if (this.editMode === 'none') {
        this.canvas.style.cursor = 'default';
      } else if (this.editMode === 'add') {
        this.canvas.style.cursor = 'crosshair';
      } else if (this.editMode === 'delete') {
        // Check if hovering over an obstacle
        const rect = this.canvas.getBoundingClientRect();
        const canvasX = event.clientX - rect.left;
        const canvasY = event.clientY - rect.top;
        const worldCoords = this._canvasToWorld(canvasX, canvasY);

        const index = WorldState.findObstacleAt(worldCoords.x, worldCoords.y);
        this.canvas.style.cursor = index >= 0 ? 'pointer' : 'not-allowed';
      }
    });
  }

  /**
   * Convert canvas coordinates to world coordinates
   */
  _canvasToWorld(canvasX, canvasY) {
    const scaleX = this.worldWidth / this.canvas.width;
    const scaleY = this.worldHeight / this.canvas.height;
    return {
      x: canvasX * scaleX,
      y: (this.canvas.height - canvasY) * scaleY // Flip Y axis
    };
  }

  _handleResize() {
    const container = this.canvas.parentElement;
    if (!container) return;

    const rect = container.getBoundingClientRect();
    const size = Math.min(rect.width, rect.height, 500);

    this.canvas.width = size;
    this.canvas.height = size;

    this._render();
  }

  /**
   * Convert world coordinates to canvas coordinates
   */
  _worldToCanvas(x, y) {
    const scaleX = this.canvas.width / this.worldWidth;
    const scaleY = this.canvas.height / this.worldHeight;
    return {
      x: x * scaleX,
      y: this.canvas.height - y * scaleY // Flip Y axis
    };
  }

  /**
   * Convert world distance to canvas distance
   */
  _worldToCanvasScale(distance) {
    return (distance / this.worldWidth) * this.canvas.width;
  }

  /**
   * Main render function
   */
  _render() {
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;

    // Clear canvas with background color
    ctx.fillStyle = `rgb(${this.background.r}, ${this.background.g}, ${this.background.b})`;
    ctx.fillRect(0, 0, width, height);

    // Draw obstacles
    if (this.showObstacles) {
      this._renderObstacles();
    }

    // Draw trails
    this._renderTrails();

    // Draw lidar rays
    if (this.showLidar && this.lidarData) {
      this._renderLidar();
    }

    // Draw turtles
    this._renderTurtles();

    // Draw TF frames
    if (this.showTF) {
      this._renderTFFrames();
    }
  }

  _renderObstacles() {
    const ctx = this.ctx;
    const obstacles = WorldState.getObstacles();

    ctx.fillStyle = 'rgba(100, 100, 100, 0.8)';
    ctx.strokeStyle = '#444';
    ctx.lineWidth = 2;

    for (const obs of obstacles) {
      const topLeft = this._worldToCanvas(obs.x, obs.y + obs.height);
      const w = this._worldToCanvasScale(obs.width);
      const h = this._worldToCanvasScale(obs.height);

      ctx.fillRect(topLeft.x, topLeft.y, w, h);
      ctx.strokeRect(topLeft.x, topLeft.y, w, h);
    }
  }

  _renderTrails() {
    const ctx = this.ctx;

    for (const trail of this.trails) {
      const p1 = this._worldToCanvas(trail.x1, trail.y1);
      const p2 = this._worldToCanvas(trail.x2, trail.y2);

      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.strokeStyle = trail.color;
      ctx.lineWidth = trail.width;
      ctx.lineCap = 'round';
      ctx.stroke();
    }
  }

  _renderTurtles() {
    const ctx = this.ctx;
    const turtleSize = this._worldToCanvasScale(1.2);

    for (const turtle of this.turtles) {
      const pos = this._worldToCanvas(turtle.x, turtle.y);

      ctx.save();
      ctx.translate(pos.x, pos.y);
      // Canvas Y is flipped, so negate angle
      ctx.rotate(-turtle.theta);

      if (this.turtleImageLoaded) {
        ctx.drawImage(
          this.turtleImage,
          -turtleSize / 2,
          -turtleSize / 2,
          turtleSize,
          turtleSize
        );
      } else {
        // Fallback: draw simple triangle
        ctx.beginPath();
        ctx.moveTo(turtleSize / 2, 0);
        ctx.lineTo(-turtleSize / 2, -turtleSize / 3);
        ctx.lineTo(-turtleSize / 2, turtleSize / 3);
        ctx.closePath();
        ctx.fillStyle = '#00ff00';
        ctx.fill();
        ctx.strokeStyle = '#006400';
        ctx.stroke();
      }

      ctx.restore();
    }
  }

  _renderLidar() {
    const ctx = this.ctx;
    const { x, y, theta, scan } = this.lidarData;

    if (!scan || !scan.ranges) return;

    const pos = this._worldToCanvas(x, y);

    ctx.strokeStyle = 'rgba(255, 0, 0, 0.3)';
    ctx.lineWidth = 1;

    const { angle_min, angle_increment, ranges } = scan;

    for (let i = 0; i < ranges.length; i++) {
      const range = ranges[i];
      if (!isFinite(range)) continue;

      const angle = theta + angle_min + i * angle_increment;
      const endX = x + Math.cos(angle) * range;
      const endY = y + Math.sin(angle) * range;
      const endPos = this._worldToCanvas(endX, endY);

      ctx.beginPath();
      ctx.moveTo(pos.x, pos.y);
      ctx.lineTo(endPos.x, endPos.y);
      ctx.stroke();

      // Draw hit point
      if (range < scan.range_max - 0.1) {
        ctx.fillStyle = 'red';
        ctx.beginPath();
        ctx.arc(endPos.x, endPos.y, 2, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  _renderTFFrames() {
    // TF frame visualization - show coordinate axes
    const ctx = this.ctx;
    const axisLength = this._worldToCanvasScale(0.5);

    for (const turtle of this.turtles) {
      const pos = this._worldToCanvas(turtle.x, turtle.y);

      ctx.save();
      ctx.translate(pos.x, pos.y);
      ctx.rotate(-turtle.theta);

      // X axis (red)
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(axisLength, 0);
      ctx.strokeStyle = 'red';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Y axis (green)
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(0, -axisLength); // Negative because canvas Y is flipped
      ctx.strokeStyle = 'green';
      ctx.stroke();

      ctx.restore();
    }
  }

  /**
   * Toggle obstacle visibility
   */
  toggleObstacles() {
    this.showObstacles = !this.showObstacles;
    this._render();
  }

  /**
   * Toggle lidar visualization
   */
  toggleLidar() {
    this.showLidar = !this.showLidar;
    this._render();
  }

  /**
   * Toggle TF frame visualization
   */
  toggleTF() {
    this.showTF = !this.showTF;
    this._render();
  }

  /**
   * Clear trails
   */
  clearTrails() {
    this.trails = [];
    this._render();
  }

  /**
   * Set edit mode
   * @param {'none' | 'add' | 'delete'} mode
   */
  setEditMode(mode) {
    this.editMode = mode;
    this.canvas.style.cursor = mode === 'none' ? 'default' : mode === 'add' ? 'crosshair' : 'pointer';
    window.dispatchEvent(new CustomEvent('canvas-edit-mode-changed', { detail: { mode } }));
  }

  /**
   * Get current edit mode
   */
  getEditMode() {
    return this.editMode;
  }

  /**
   * Set size for new obstacles
   */
  setObstacleSize(size) {
    this.newObstacleSize = Math.max(0.3, Math.min(3, size));
  }

  /**
   * Randomize obstacles
   */
  randomizeObstacles(count = 5) {
    WorldState.randomizeObstacles(count);
  }

  /**
   * Clear all obstacles (keep walls)
   */
  clearObstacles() {
    WorldState.clearNonWallObstacles();
  }

  /**
   * Reset to default obstacles
   */
  resetObstacles() {
    WorldState.resetObstacles();
  }
}
