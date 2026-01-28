import cytoscape from 'cytoscape';
import { SimDDS } from '../core/SimDDS.js';

/**
 * Graph - rqt_graph visualization using Cytoscape.js
 */
export class Graph {
  constructor(containerId = 'graph-container') {
    this.container = document.getElementById(containerId);
    this.cy = null;
    this.updateInterval = null;

    this._initCytoscape();
  }

  _initCytoscape() {
    // Defer cytoscape initialization - it fails on hidden containers
    this.cy = null;
    this.initialized = false;
    this.lastGraphSignature = '';
  }

  _ensureInitialized() {
    if (this.initialized) return;

    this.cy = cytoscape({
      container: this.container,
      elements: [],
      style: [
        {
          selector: 'node[type="node"]',
          style: {
            'background-color': '#4a90d9',
            'label': 'data(label)',
            'text-valign': 'center',
            'text-halign': 'center',
            'color': '#fff',
            'font-size': '12px',
            'width': '80px',
            'height': '40px',
            'shape': 'round-rectangle',
            'text-wrap': 'wrap',
            'text-max-width': '70px'
          }
        },
        {
          selector: 'node[type="topic"]',
          style: {
            'background-color': '#6c757d',
            'label': 'data(label)',
            'text-valign': 'center',
            'text-halign': 'center',
            'color': '#fff',
            'font-size': '10px',
            'width': '100px',
            'height': '30px',
            'shape': 'rectangle',
            'text-wrap': 'wrap',
            'text-max-width': '90px'
          }
        },
        {
          selector: 'edge',
          style: {
            'width': 2,
            'line-color': '#aaa',
            'target-arrow-color': '#aaa',
            'target-arrow-shape': 'triangle',
            'curve-style': 'bezier'
          }
        },
        {
          selector: 'edge[type="publish"]',
          style: {
            'line-color': '#28a745',
            'target-arrow-color': '#28a745'
          }
        },
        {
          selector: 'edge[type="subscribe"]',
          style: {
            'line-color': '#dc3545',
            'target-arrow-color': '#dc3545'
          }
        }
      ],
      layout: {
        name: 'preset'
      }
    });
    this.initialized = true;
  }

  /**
   * Update the graph with current ROS2 state
   */
  update() {
    if (!this.initialized || !this.cy) return;

    const nodes = SimDDS.getNodes();
    const topics = SimDDS.getTopics();

    // Build graph elements
    const elements = [];
    const nodeSet = new Set();
    const topicSet = new Set();
    const elementIds = [];

    // Add node nodes
    for (const nodeName of nodes) {
      const id = `node_${nodeName}`;
      nodeSet.add(id);
      elementIds.push(id);
      elements.push({
        data: {
          id,
          label: nodeName.replace(/^\//, ''),
          type: 'node'
        }
      });
    }

    // Add topic nodes and edges
    for (const topic of topics) {
      const topicId = `topic_${topic.name}`;

      // Only add topic if it has publishers or subscribers
      if (topic.publishers.length === 0 && topic.subscribers.length === 0) {
        continue;
      }

      if (!topicSet.has(topicId)) {
        topicSet.add(topicId);
        elementIds.push(topicId);
        elements.push({
          data: {
            id: topicId,
            label: topic.name,
            type: 'topic'
          }
        });
      }

      // Add edges from publishers to topic
      for (const pub of topic.publishers) {
        const nodeId = `node_${pub.nodeId}`;
        if (nodeSet.has(nodeId)) {
          const edgeId = `${nodeId}_to_${topicId}`;
          elementIds.push(edgeId);
          elements.push({
            data: {
              id: edgeId,
              source: nodeId,
              target: topicId,
              type: 'publish'
            }
          });
        }
      }

      // Add edges from topic to subscribers
      for (const sub of topic.subscribers) {
        const nodeId = `node_${sub.nodeId}`;
        if (nodeSet.has(nodeId)) {
          const edgeId = `${topicId}_to_${nodeId}`;
          elementIds.push(edgeId);
          elements.push({
            data: {
              id: edgeId,
              source: topicId,
              target: nodeId,
              type: 'subscribe'
            }
          });
        }
      }
    }

    // Create a signature of current graph structure
    const signature = elementIds.sort().join(',');

    // Only rebuild if structure changed
    if (signature === this.lastGraphSignature) {
      return;
    }
    this.lastGraphSignature = signature;

    // Ensure container is sized properly
    this.cy.resize();

    // Clear and rebuild graph
    this.cy.elements().remove();
    this.cy.add(elements);

    // Apply layout only if there are elements
    if (elements.length > 0) {
      this.cy.layout({
        name: 'cose',
        animate: false,
        nodeDimensionsIncludeLabels: true,
        nodeRepulsion: 4000,
        idealEdgeLength: 80,
        padding: 50
      }).run();

      // Fit to view with padding
      this.cy.fit(undefined, 50);
      this.cy.center();
    }
  }

  /**
   * Start auto-updating
   */
  startAutoUpdate(intervalMs = 1000) {
    this.stopAutoUpdate();
    this.update();
    this.updateInterval = setInterval(() => this.update(), intervalMs);
  }

  /**
   * Stop auto-updating
   */
  stopAutoUpdate() {
    if (this.updateInterval) {
      clearInterval(this.updateInterval);
      this.updateInterval = null;
    }
  }

  /**
   * Show the graph
   */
  show() {
    // Hide canvas to give graph full space
    const canvasContainer = document.getElementById('canvas-container');
    canvasContainer?.classList.add('hidden');

    this.container.classList.remove('hidden');

    // Initialize cytoscape now that container is visible
    setTimeout(() => {
      this._ensureInitialized();
      if (this.cy) {
        this.cy.resize();
      }
      this.startAutoUpdate();
      // Additional resize and center after layout settles
      setTimeout(() => {
        if (this.cy) {
          this.cy.resize();
          this.cy.fit(undefined, 50);
          this.cy.center();
        }
      }, 200);
    }, 100);
  }

  /**
   * Hide the graph
   */
  hide() {
    this.container.classList.add('hidden');
    this.stopAutoUpdate();

    // Show canvas again
    const canvasContainer = document.getElementById('canvas-container');
    canvasContainer?.classList.remove('hidden');
  }

  /**
   * Toggle visibility
   */
  toggle() {
    if (this.container.classList.contains('hidden')) {
      this.show();
    } else {
      this.hide();
    }
  }

  /**
   * Destroy the graph
   */
  destroy() {
    this.stopAutoUpdate();
    if (this.cy) {
      this.cy.destroy();
      this.cy = null;
    }
    this.initialized = false;
  }
}
