/**
 * Layout - Manages responsive layout for terminal/canvas/graph panels
 */
export class Layout {
  constructor() {
    this.visualizationPanel = document.getElementById('visualization-panel');
    this.terminalPanel = document.getElementById('terminal-panel');
    this.canvasContainer = document.getElementById('canvas-container');
    this.mapContainer = document.getElementById('map-container');
    this.graphContainer = document.getElementById('graph-container');
    this.consolePanel = document.getElementById('console-panel');

    this.resizing = false;
    this.currentLayout = 'desktop';

    this._detectLayout();
    this._setupResizeObserver();
    this._setupMobileSwipe();
  }

  _detectLayout() {
    const width = window.innerWidth;

    if (width < 768) {
      this.currentLayout = 'mobile';
      document.body.classList.add('mobile-layout');
      document.body.classList.remove('tablet-layout', 'desktop-layout');
    } else if (width < 1024) {
      this.currentLayout = 'tablet';
      document.body.classList.add('tablet-layout');
      document.body.classList.remove('mobile-layout', 'desktop-layout');
    } else {
      this.currentLayout = 'desktop';
      document.body.classList.add('desktop-layout');
      document.body.classList.remove('mobile-layout', 'tablet-layout');
    }
  }

  _setupResizeObserver() {
    window.addEventListener('resize', () => {
      this._detectLayout();
      this._triggerResize();
    });
  }

  _setupMobileSwipe() {
    if (!this.terminalPanel) return;

    let startX = 0;
    let startY = 0;

    this.terminalPanel.addEventListener('touchstart', (e) => {
      startX = e.touches[0].clientX;
      startY = e.touches[0].clientY;
    }, { passive: true });

    this.terminalPanel.addEventListener('touchend', (e) => {
      const endX = e.changedTouches[0].clientX;
      const endY = e.changedTouches[0].clientY;
      const diffX = endX - startX;
      const diffY = endY - startY;

      // Only handle horizontal swipes
      if (Math.abs(diffX) > Math.abs(diffY) && Math.abs(diffX) > 50) {
        // Emit swipe event for mobile terminal switching
        const event = new CustomEvent('terminal-swipe', {
          detail: { direction: diffX > 0 ? 'right' : 'left' }
        });
        this.terminalPanel.dispatchEvent(event);
      }
    }, { passive: true });
  }

  _triggerResize() {
    // Dispatch resize event for components to handle
    window.dispatchEvent(new Event('layout-resize'));
  }

  /**
   * Show the canvas panel
   */
  showCanvas() {
    this.canvasContainer?.classList.remove('hidden');
  }

  /**
   * Hide the canvas panel
   */
  hideCanvas() {
    this.canvasContainer?.classList.add('hidden');
  }

  /**
   * Toggle the map panel
   */
  toggleMap() {
    this.mapContainer?.classList.toggle('hidden');
    this._triggerResize();
  }

  /**
   * Show the map panel
   */
  showMap() {
    this.mapContainer?.classList.remove('hidden');
    this._triggerResize();
  }

  /**
   * Hide the map panel
   */
  hideMap() {
    this.mapContainer?.classList.add('hidden');
    this._triggerResize();
  }

  /**
   * Toggle the graph panel
   */
  toggleGraph() {
    this.graphContainer?.classList.toggle('hidden');
    this._triggerResize();
  }

  /**
   * Show the graph panel
   */
  showGraph() {
    this.graphContainer?.classList.remove('hidden');
    this._triggerResize();
  }

  /**
   * Hide the graph panel
   */
  hideGraph() {
    this.graphContainer?.classList.add('hidden');
    this._triggerResize();
  }

  /**
   * Toggle the console panel
   */
  toggleConsole() {
    this.consolePanel?.classList.toggle('hidden');
    this._triggerResize();
  }

  /**
   * Show the console panel
   */
  showConsole() {
    this.consolePanel?.classList.remove('hidden');
    this._triggerResize();
  }

  /**
   * Hide the console panel
   */
  hideConsole() {
    this.consolePanel?.classList.add('hidden');
    this._triggerResize();
  }

  /**
   * Get current layout mode
   */
  getLayout() {
    return this.currentLayout;
  }

  /**
   * Check if mobile layout
   */
  isMobile() {
    return this.currentLayout === 'mobile';
  }

  /**
   * Check if tablet layout
   */
  isTablet() {
    return this.currentLayout === 'tablet';
  }

  /**
   * Check if desktop layout
   */
  isDesktop() {
    return this.currentLayout === 'desktop';
  }
}
