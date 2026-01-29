import { describe, it, expect, beforeEach, vi, afterEach } from 'vitest';
import { ProcessManagerClass } from './ProcessManager.js';
import { Events } from './Events.js';

describe('ProcessManager', () => {
  let processManager;
  let mockNode;

  beforeEach(() => {
    // Create a fresh ProcessManager instance for each test
    processManager = new ProcessManagerClass();

    // Mock the logger to avoid LogManager dependency
    processManager.logger = {
      debug: vi.fn(),
      info: vi.fn(),
      warn: vi.fn(),
      error: vi.fn()
    };

    // Create mock node
    mockNode = {
      fullName: '/test_node',
      name: 'test_node',
      start: vi.fn(),
      destroy: vi.fn()
    };

    // Mock window.dispatchEvent
    vi.spyOn(window, 'dispatchEvent').mockImplementation(() => {});
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  describe('spawn', () => {
    it('creates a process and returns process ID', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'turtlesim', 'turtlesim_node');

      expect(processId).toBeDefined();
      expect(processId).toMatch(/^proc_\d+$/);
    });

    it('starts the node', () => {
      processManager.spawn(mockNode, 'term_1', 'turtlesim', 'turtlesim_node');

      expect(mockNode.start).toHaveBeenCalled();
    });

    it('stores process info', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'turtlesim', 'turtlesim_node');

      const proc = processManager.getProcess(processId);
      expect(proc).toBeDefined();
      expect(proc.node).toBe(mockNode);
      expect(proc.terminalId).toBe('term_1');
      expect(proc.package).toBe('turtlesim');
      expect(proc.executable).toBe('turtlesim_node');
      expect(proc.startTime).toBeDefined();
    });

    it('dispatches PROCESS_STARTED event', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'turtlesim', 'turtlesim_node');

      expect(window.dispatchEvent).toHaveBeenCalledWith(
        expect.objectContaining({
          type: Events.PROCESS_STARTED,
          detail: { processId, terminalId: 'term_1', nodeName: '/test_node' }
        })
      );
    });

    it('generates unique process IDs', () => {
      const id1 = processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      const mockNode2 = { ...mockNode, fullName: '/node2', start: vi.fn(), destroy: vi.fn() };
      const id2 = processManager.spawn(mockNode2, 'term_1', 'pkg', 'exe');

      expect(id1).not.toBe(id2);
    });
  });

  describe('kill', () => {
    it('destroys the node and removes process', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      const result = processManager.kill(processId);

      expect(result).toBe(true);
      expect(mockNode.destroy).toHaveBeenCalled();
      expect(processManager.getProcess(processId)).toBeNull();
    });

    it('returns false for non-existent process', () => {
      const result = processManager.kill('nonexistent_id');
      expect(result).toBe(false);
    });

    it('dispatches PROCESS_STOPPED event', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      processManager.kill(processId);

      expect(window.dispatchEvent).toHaveBeenCalledWith(
        expect.objectContaining({
          type: Events.PROCESS_STOPPED,
          detail: { processId, terminalId: 'term_1', nodeName: '/test_node' }
        })
      );
    });
  });

  describe('killByNodeName', () => {
    it('kills process by node name', () => {
      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      const result = processManager.killByNodeName('/test_node');

      expect(result).toBe(true);
      expect(mockNode.destroy).toHaveBeenCalled();
    });

    it('returns false for non-existent node name', () => {
      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      const result = processManager.killByNodeName('/nonexistent');

      expect(result).toBe(false);
    });
  });

  describe('killByTerminal', () => {
    it('kills all processes for a terminal', () => {
      const mockNode2 = { ...mockNode, fullName: '/node2', start: vi.fn(), destroy: vi.fn() };
      const mockNode3 = { ...mockNode, fullName: '/node3', start: vi.fn(), destroy: vi.fn() };

      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');
      processManager.spawn(mockNode2, 'term_1', 'pkg', 'exe');
      processManager.spawn(mockNode3, 'term_2', 'pkg', 'exe'); // Different terminal

      const count = processManager.killByTerminal('term_1');

      expect(count).toBe(2);
      expect(mockNode.destroy).toHaveBeenCalled();
      expect(mockNode2.destroy).toHaveBeenCalled();
      expect(mockNode3.destroy).not.toHaveBeenCalled();
    });

    it('returns 0 when no processes for terminal', () => {
      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      const count = processManager.killByTerminal('term_2');

      expect(count).toBe(0);
    });
  });

  describe('getProcess / getProcessByNodeName', () => {
    it('getProcess returns process info by ID', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      const proc = processManager.getProcess(processId);

      expect(proc).toBeDefined();
      expect(proc.node).toBe(mockNode);
    });

    it('getProcess returns null for non-existent ID', () => {
      expect(processManager.getProcess('nonexistent')).toBeNull();
    });

    it('getProcessByNodeName returns process info with processId', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      const proc = processManager.getProcessByNodeName('/test_node');

      expect(proc).toBeDefined();
      expect(proc.processId).toBe(processId);
      expect(proc.node).toBe(mockNode);
    });

    it('getProcessByNodeName returns null for non-existent name', () => {
      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      expect(processManager.getProcessByNodeName('/nonexistent')).toBeNull();
    });
  });

  describe('isNodeRunning', () => {
    it('returns true when node is running', () => {
      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      expect(processManager.isNodeRunning('/test_node')).toBe(true);
    });

    it('returns false when node is not running', () => {
      expect(processManager.isNodeRunning('/test_node')).toBe(false);
    });

    it('returns false after node is killed', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');
      processManager.kill(processId);

      expect(processManager.isNodeRunning('/test_node')).toBe(false);
    });
  });

  describe('getAllProcesses', () => {
    it('returns empty array when no processes', () => {
      expect(processManager.getAllProcesses()).toEqual([]);
    });

    it('returns all processes with processId', () => {
      const mockNode2 = { ...mockNode, fullName: '/node2', start: vi.fn(), destroy: vi.fn() };

      const id1 = processManager.spawn(mockNode, 'term_1', 'pkg1', 'exe1');
      const id2 = processManager.spawn(mockNode2, 'term_2', 'pkg2', 'exe2');

      const all = processManager.getAllProcesses();

      expect(all).toHaveLength(2);
      expect(all.map(p => p.processId)).toContain(id1);
      expect(all.map(p => p.processId)).toContain(id2);
    });
  });

  describe('getProcessesByTerminal', () => {
    it('returns processes for specific terminal', () => {
      const mockNode2 = { ...mockNode, fullName: '/node2', start: vi.fn(), destroy: vi.fn() };
      const mockNode3 = { ...mockNode, fullName: '/node3', start: vi.fn(), destroy: vi.fn() };

      const id1 = processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');
      const id2 = processManager.spawn(mockNode2, 'term_1', 'pkg', 'exe');
      processManager.spawn(mockNode3, 'term_2', 'pkg', 'exe');

      const procs = processManager.getProcessesByTerminal('term_1');

      expect(procs).toHaveLength(2);
      expect(procs.map(p => p.processId)).toContain(id1);
      expect(procs.map(p => p.processId)).toContain(id2);
    });

    it('returns empty array for terminal with no processes', () => {
      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');

      expect(processManager.getProcessesByTerminal('term_2')).toEqual([]);
    });
  });

  describe('getProcessCount', () => {
    it('returns 0 when no processes', () => {
      expect(processManager.getProcessCount()).toBe(0);
    });

    it('returns correct count', () => {
      const mockNode2 = { ...mockNode, fullName: '/node2', start: vi.fn(), destroy: vi.fn() };

      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');
      processManager.spawn(mockNode2, 'term_1', 'pkg', 'exe');

      expect(processManager.getProcessCount()).toBe(2);
    });

    it('updates count after kill', () => {
      const processId = processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');
      expect(processManager.getProcessCount()).toBe(1);

      processManager.kill(processId);
      expect(processManager.getProcessCount()).toBe(0);
    });
  });

  describe('killAll', () => {
    it('kills all processes', () => {
      const mockNode2 = { ...mockNode, fullName: '/node2', start: vi.fn(), destroy: vi.fn() };
      const mockNode3 = { ...mockNode, fullName: '/node3', start: vi.fn(), destroy: vi.fn() };

      processManager.spawn(mockNode, 'term_1', 'pkg', 'exe');
      processManager.spawn(mockNode2, 'term_1', 'pkg', 'exe');
      processManager.spawn(mockNode3, 'term_2', 'pkg', 'exe');

      processManager.killAll();

      expect(processManager.getProcessCount()).toBe(0);
      expect(mockNode.destroy).toHaveBeenCalled();
      expect(mockNode2.destroy).toHaveBeenCalled();
      expect(mockNode3.destroy).toHaveBeenCalled();
    });

    it('handles empty process list', () => {
      expect(() => processManager.killAll()).not.toThrow();
    });
  });
});
