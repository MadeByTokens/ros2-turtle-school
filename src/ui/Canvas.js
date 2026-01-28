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

    // Map overlay options
    this.showMap = false;
    this.mapOpacity = 0.5;
    this.mapData = null;
    this.mapInfo = null;
    this.mapCanvas = null; // Off-screen canvas for map rendering

    // Resize debounce guard
    this._resizeScheduled = false;

    // Edit mode: 'none', 'add', 'delete'
    this.editMode = 'none';
    this.newObstacleSize = 1.0;

    // Current state
    this.turtles = [];
    this.trails = [];
    this.background = { r: 69, g: 86, b: 255 };
    this.lidarData = null;
    this.hasTurtlesim = false;

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
      this.hasTurtlesim = turtles && turtles.length > 0;
      this._render();
    });

    // Listen for lidar data
    window.addEventListener('lidar-update', (event) => {
      this.lidarData = event.detail;
      if (this.showLidar) {
        this._render();
      }
    });

    // Listen for map updates
    window.addEventListener('map-update', (event) => {
      const { map, info } = event.detail;
      this.mapData = map;
      this.mapInfo = info;
      this._updateMapCanvas();
      if (this.showMap) {
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
    if (this._resizeScheduled) return;
    this._resizeScheduled = true;
    requestAnimationFrame(() => {
      this._resizeScheduled = false;
      const container = this.canvas.parentElement;
      if (!container) return;

      const rect = container.getBoundingClientRect();
      // Don't resize if container is hidden (rect would be all zeros)
      if (rect.width === 0 || rect.height === 0) return;

      const size = Math.min(rect.width, rect.height, 500);

      this.canvas.width = size;
      this.canvas.height = size;

      this._render();
    });
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
   * Update the off-screen map canvas when map data changes
   */
  _updateMapCanvas() {
    if (!this.mapData || !this.mapInfo) return;

    const { width, height, resolution, origin } = this.mapInfo;
    if (!width || !height) return;

    // Create or resize off-screen canvas
    if (!this.mapCanvas) {
      this.mapCanvas = document.createElement('canvas');
    }
    this.mapCanvas.width = width;
    this.mapCanvas.height = height;

    const mapCtx = this.mapCanvas.getContext('2d');
    const imageData = mapCtx.createImageData(width, height);

    // Render occupancy grid data to image with distinct colors
    // Note: Grid data is stored with Y=0 at bottom (world space)
    // Image pixels have Y=0 at top, so we flip Y when writing
    for (let gridY = 0; gridY < height; gridY++) {
      for (let gridX = 0; gridX < width; gridX++) {
        const gridIndex = gridY * width + gridX;
        const value = this.mapData[gridIndex];

        // Flip Y: grid row 0 (world bottom) -> image row (height-1) (image bottom)
        const imageY = height - 1 - gridY;
        const pixelIndex = (imageY * width + gridX) * 4;

        let r, g, b;

        if (value === -1) {
          // Unknown - transparent blue-gray (will blend with background)
          r = 100;
          g = 100;
          b = 120;
        } else if (value <= 10) {
          // Free space - white/light green tint
          r = 220;
          g = 255;
          b = 220;
        } else if (value >= 65) {
          // Definitely occupied - strong red/dark
          const intensity = Math.min(100, value);
          r = 180 - intensity;  // Gets darker red as more occupied
          g = 20;
          b = 20;
        } else {
          // Partially occupied - orange to red gradient (11-64)
          const t = (value - 10) / 55; // 0 to 1
          r = Math.round(255 - t * 75);  // 255 -> 180
          g = Math.round(180 - t * 160); // 180 -> 20
          b = Math.round(100 - t * 80);  // 100 -> 20
        }

        imageData.data[pixelIndex] = r;
        imageData.data[pixelIndex + 1] = g;
        imageData.data[pixelIndex + 2] = b;
        imageData.data[pixelIndex + 3] = 255;
      }
    }

    mapCtx.putImageData(imageData, 0, 0);
  }

  /**
   * Render map overlay on canvas
   */
  _renderMapOverlay() {
    if (!this.mapCanvas || !this.mapInfo) return;

    const ctx = this.ctx;
    const { width: mapWidth, height: mapHeight, resolution, origin } = this.mapInfo;

    // Save context state
    ctx.save();
    ctx.globalAlpha = this.mapOpacity;

    // Disable image smoothing for crisp pixel rendering
    ctx.imageSmoothingEnabled = false;

    // The map covers the entire world (0,0) to (worldWidth, worldHeight)
    // Draw it to fill the entire canvas
    ctx.drawImage(this.mapCanvas, 0, 0, this.canvas.width, this.canvas.height);

    // Restore context state
    ctx.restore();
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

    // Draw map overlay (before obstacles, after background)
    if (this.showMap) {
      this._renderMapOverlay();
    }

    // Draw obstacles
    if (this.showObstacles) {
      this._renderObstacles();
    }

    // If no turtlesim running, show empty state hint
    if (!this.hasTurtlesim) {
      this._renderEmptyState();
      return;
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

  /**
   * Render empty state hint when no turtlesim is running
   */
  _renderEmptyState() {
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;

    // Semi-transparent overlay
    ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
    ctx.fillRect(0, 0, width, height);

    // Text
    ctx.fillStyle = '#ffffff';
    ctx.font = '14px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    const text = 'Run: ros2 run turtlesim turtlesim_node';
    ctx.fillText(text, width / 2, height / 2);

    ctx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif';
    ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
    ctx.fillText('to start the simulation', width / 2, height / 2 + 20);
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

  /**
   * Toggle map overlay visibility
   */
  toggleMap() {
    this.showMap = !this.showMap;
    this._render();
    return this.showMap;
  }

  /**
   * Set map overlay opacity
   * @param {number} opacity - Opacity value between 0 and 1
   */
  setMapOpacity(opacity) {
    this.mapOpacity = Math.max(0, Math.min(1, opacity));
    if (this.showMap) {
      this._render();
    }
  }

  /**
   * Get current map opacity
   */
  getMapOpacity() {
    return this.mapOpacity;
  }

  /**
   * Check if map is currently visible
   */
  isMapVisible() {
    return this.showMap;
  }
}
