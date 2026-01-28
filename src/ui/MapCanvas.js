/**
 * MapCanvas - Occupancy grid visualization for SLAM
 */
export class MapCanvas {
  constructor(canvasId = 'map-canvas') {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');

    // Map data
    this.mapData = null;
    this.mapInfo = null;

    // Robot position
    this.robotX = 0;
    this.robotY = 0;
    this.robotTheta = 0;

    // Set up event listeners
    this._setupEventListeners();

    // Initial render
    this._render();
  }

  _setupEventListeners() {
    // Listen for map updates
    window.addEventListener('map-update', (event) => {
      const { map, info } = event.detail;
      this.mapData = map;
      this.mapInfo = info;
      this._render();
    });

    // Listen for robot pose updates
    window.addEventListener('robot-pose-update', (event) => {
      const { x, y, theta } = event.detail;
      this.robotX = x;
      this.robotY = y;
      this.robotTheta = theta;
      this._render();
    });

    // Handle resize
    window.addEventListener('resize', () => this._handleResize());
    window.addEventListener('layout-resize', () => this._handleResize());

    // Initial size
    this._handleResize();
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
   * Convert map coordinates to canvas coordinates
   */
  _mapToCanvas(mapX, mapY) {
    if (!this.mapInfo) return { x: 0, y: 0 };

    const { width, height, resolution, origin } = this.mapInfo;

    // World coordinates
    const worldX = origin.position.x + mapX * resolution;
    const worldY = origin.position.y + mapY * resolution;

    // Scale to canvas
    const mapWorldWidth = width * resolution;
    const mapWorldHeight = height * resolution;

    const scaleX = this.canvas.width / mapWorldWidth;
    const scaleY = this.canvas.height / mapWorldHeight;

    return {
      x: (worldX - origin.position.x) * scaleX,
      y: this.canvas.height - (worldY - origin.position.y) * scaleY
    };
  }

  /**
   * Convert world coordinates to canvas coordinates
   */
  _worldToCanvas(worldX, worldY) {
    if (!this.mapInfo) {
      // Default to 11x11 world
      const scale = this.canvas.width / 11;
      return {
        x: worldX * scale,
        y: this.canvas.height - worldY * scale
      };
    }

    const { width, height, resolution, origin } = this.mapInfo;
    const mapWorldWidth = width * resolution;
    const mapWorldHeight = height * resolution;

    const scaleX = this.canvas.width / mapWorldWidth;
    const scaleY = this.canvas.height / mapWorldHeight;

    return {
      x: (worldX - origin.position.x) * scaleX,
      y: this.canvas.height - (worldY - origin.position.y) * scaleY
    };
  }

  /**
   * Main render function
   */
  _render() {
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;

    // Clear with gray (unknown)
    ctx.fillStyle = '#808080';
    ctx.fillRect(0, 0, width, height);

    // Draw occupancy grid
    if (this.mapData && this.mapInfo) {
      this._renderOccupancyGrid();
    }

    // Draw robot
    this._renderRobot();
  }

  _renderOccupancyGrid() {
    const ctx = this.ctx;
    const { width: mapWidth, height: mapHeight, resolution, origin } = this.mapInfo;
    const data = this.mapData;

    // Calculate cell size on canvas
    const mapWorldWidth = mapWidth * resolution;
    const mapWorldHeight = mapHeight * resolution;
    const cellWidth = this.canvas.width / mapWidth;
    const cellHeight = this.canvas.height / mapHeight;

    // Create ImageData for efficient rendering
    const imageData = ctx.createImageData(this.canvas.width, this.canvas.height);
    const pixels = imageData.data;

    for (let my = 0; my < mapHeight; my++) {
      for (let mx = 0; mx < mapWidth; mx++) {
        const idx = my * mapWidth + mx;
        const value = data[idx];

        // Calculate canvas position (flip Y)
        const canvasY = mapHeight - 1 - my;

        // Get color based on occupancy value
        let r, g, b;
        if (value === -1) {
          // Unknown - gray
          r = g = b = 128;
        } else if (value === 0) {
          // Free - white
          r = g = b = 255;
        } else {
          // Occupied - black (scale from gray to black based on value)
          const intensity = Math.max(0, 255 - (value * 2.55));
          r = g = b = intensity;
        }

        // Fill cell in image data
        for (let py = 0; py < Math.ceil(cellHeight); py++) {
          for (let px = 0; px < Math.ceil(cellWidth); px++) {
            const pixelX = Math.floor(mx * cellWidth) + px;
            const pixelY = Math.floor(canvasY * cellHeight) + py;

            if (pixelX < this.canvas.width && pixelY < this.canvas.height) {
              const pixelIdx = (pixelY * this.canvas.width + pixelX) * 4;
              pixels[pixelIdx] = r;
              pixels[pixelIdx + 1] = g;
              pixels[pixelIdx + 2] = b;
              pixels[pixelIdx + 3] = 255;
            }
          }
        }
      }
    }

    ctx.putImageData(imageData, 0, 0);
  }

  _renderRobot() {
    const ctx = this.ctx;
    const pos = this._worldToCanvas(this.robotX, this.robotY);
    const robotSize = 15;

    ctx.save();
    ctx.translate(pos.x, pos.y);
    ctx.rotate(-this.robotTheta);

    // Draw robot as arrow
    ctx.beginPath();
    ctx.moveTo(robotSize, 0);
    ctx.lineTo(-robotSize / 2, -robotSize / 2);
    ctx.lineTo(-robotSize / 3, 0);
    ctx.lineTo(-robotSize / 2, robotSize / 2);
    ctx.closePath();

    ctx.fillStyle = 'rgba(0, 150, 255, 0.8)';
    ctx.fill();
    ctx.strokeStyle = '#0066cc';
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.restore();
  }

  /**
   * Update map data
   */
  updateMap(data, info) {
    this.mapData = data;
    this.mapInfo = info;
    this._render();
  }

  /**
   * Update robot pose
   */
  updateRobotPose(x, y, theta) {
    this.robotX = x;
    this.robotY = y;
    this.robotTheta = theta;
    this._render();
  }

  /**
   * Clear the map
   */
  clear() {
    this.mapData = null;
    this.mapInfo = null;
    this._render();
  }
}
