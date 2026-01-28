import { LogManager } from './LogManager.js';

/**
 * ProcessManager - Manages running nodes and their lifecycle
 */
class ProcessManagerClass {
  constructor() {
    this.processes = new Map(); // processId -> { node, terminalId, package, executable }
    this.nextProcessId = 1;
    this.logger = LogManager.getLogger('/process_manager');
  }

  /**
   * Spawn a new node
   * @param {Node} node - The node instance to spawn
   * @param {string} terminalId - ID of the terminal that spawned this node
   * @param {string} packageName - Package name
   * @param {string} executable - Executable name
   * @returns {string} Process ID
   */
  spawn(node, terminalId, packageName, executable) {
    const processId = `proc_${this.nextProcessId++}`;

    this.processes.set(processId, {
      node,
      terminalId,
      package: packageName,
      executable,
      startTime: Date.now()
    });

    node.start();
    this.logger.info(`Spawned ${packageName}/${executable} as ${node.fullName} (pid: ${processId})`);

    return processId;
  }

  /**
   * Kill a process by ID
   * @param {string} processId - Process ID to kill
   * @returns {boolean} True if process was killed
   */
  kill(processId) {
    const proc = this.processes.get(processId);
    if (!proc) {
      return false;
    }

    proc.node.destroy();
    this.processes.delete(processId);
    this.logger.info(`Killed process ${processId}`);
    return true;
  }

  /**
   * Kill a process by node name
   * @param {string} nodeName - Full node name
   * @returns {boolean} True if process was killed
   */
  killByNodeName(nodeName) {
    for (const [processId, proc] of this.processes) {
      if (proc.node.fullName === nodeName) {
        return this.kill(processId);
      }
    }
    return false;
  }

  /**
   * Kill all processes owned by a terminal
   * @param {string} terminalId - Terminal ID
   * @returns {number} Number of processes killed
   */
  killByTerminal(terminalId) {
    let count = 0;
    const toKill = [];

    for (const [processId, proc] of this.processes) {
      if (proc.terminalId === terminalId) {
        toKill.push(processId);
      }
    }

    for (const processId of toKill) {
      if (this.kill(processId)) {
        count++;
      }
    }

    return count;
  }

  /**
   * Get process info by ID
   * @param {string} processId - Process ID
   * @returns {Object|null} Process info
   */
  getProcess(processId) {
    return this.processes.get(processId) || null;
  }

  /**
   * Get process by node name
   * @param {string} nodeName - Full node name
   * @returns {Object|null} Process info with processId
   */
  getProcessByNodeName(nodeName) {
    for (const [processId, proc] of this.processes) {
      if (proc.node.fullName === nodeName) {
        return { processId, ...proc };
      }
    }
    return null;
  }

  /**
   * Get all processes for a terminal
   * @param {string} terminalId - Terminal ID
   * @returns {Array} Array of process info
   */
  getProcessesByTerminal(terminalId) {
    const result = [];
    for (const [processId, proc] of this.processes) {
      if (proc.terminalId === terminalId) {
        result.push({ processId, ...proc });
      }
    }
    return result;
  }

  /**
   * Get all running processes
   * @returns {Array} Array of process info
   */
  getAllProcesses() {
    const result = [];
    for (const [processId, proc] of this.processes) {
      result.push({ processId, ...proc });
    }
    return result;
  }

  /**
   * Check if a node name is already running
   * @param {string} nodeName - Node name to check
   * @returns {boolean}
   */
  isNodeRunning(nodeName) {
    for (const [, proc] of this.processes) {
      if (proc.node.fullName === nodeName) {
        return true;
      }
    }
    return false;
  }

  /**
   * Get count of running processes
   * @returns {number}
   */
  getProcessCount() {
    return this.processes.size;
  }

  /**
   * Kill all processes
   */
  killAll() {
    const toKill = Array.from(this.processes.keys());
    for (const processId of toKill) {
      this.kill(processId);
    }
  }
}

// Singleton instance
export const ProcessManager = new ProcessManagerClass();
