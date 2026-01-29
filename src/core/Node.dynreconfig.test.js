import { describe, it, expect, beforeEach, vi } from 'vitest';

// Mock SimDDS
vi.mock('./SimDDS.js', () => ({
  SimDDS: {
    registerNode: vi.fn(),
    unregisterNode: vi.fn(),
    createPublisher: vi.fn(() => ({ id: 'pub1', publish: vi.fn(), destroy: vi.fn() })),
    createSubscription: vi.fn(() => ({ id: 'sub1', destroy: vi.fn() })),
    createService: vi.fn(() => ({ id: 'srv1', destroy: vi.fn() })),
    createActionServer: vi.fn(() => ({ id: 'act1', destroy: vi.fn() })),
  },
}));

// Mock LogManager
vi.mock('./LogManager.js', () => ({
  LogManager: {
    getLogger: () => ({
      debug: vi.fn(), info: vi.fn(), warn: vi.fn(), error: vi.fn(), fatal: vi.fn(),
    }),
  },
}));

// Mock ServiceContainer
vi.mock('./ServiceContainer.js', () => ({
  ServiceContainer: { get: () => null, register: vi.fn() },
}));

import { Node } from './Node.js';

describe('Dynamic Reconfigure Callbacks', () => {
  let node;

  beforeEach(() => {
    node = new Node('test_node', {
      parameters: {
        speed: 1.0,
        name: 'default',
        enabled: true,
      },
    });
  });

  describe('addOnSetParametersCallback', () => {
    it('registers a callback and returns an ID', () => {
      const id = node.addOnSetParametersCallback(() => ({ successful: true, reason: '' }));
      expect(typeof id).toBe('number');
      expect(id).toBe(0);
    });

    it('returns incrementing IDs', () => {
      const id1 = node.addOnSetParametersCallback(() => ({ successful: true, reason: '' }));
      const id2 = node.addOnSetParametersCallback(() => ({ successful: true, reason: '' }));
      expect(id2).toBe(id1 + 1);
    });
  });

  describe('setParameters', () => {
    it('sets parameters when no callbacks registered', () => {
      const result = node.setParameters({ speed: 2.0 });
      expect(result.successful).toBe(true);
      expect(node.getParameter('speed')).toBe(2.0);
    });

    it('returns failure for undeclared parameters', () => {
      const result = node.setParameters({ nonexistent: 42 });
      expect(result.successful).toBe(false);
      expect(result.reason).toContain('not declared');
    });

    it('accepts changes when callback approves', () => {
      node.addOnSetParametersCallback((params) => {
        if ('speed' in params && params.speed > 0) {
          return { successful: true, reason: '' };
        }
        return { successful: false, reason: 'speed must be positive' };
      });

      const result = node.setParameters({ speed: 5.0 });
      expect(result.successful).toBe(true);
      expect(node.getParameter('speed')).toBe(5.0);
    });

    it('rejects changes when callback rejects', () => {
      node.addOnSetParametersCallback((params) => {
        if ('speed' in params && params.speed <= 0) {
          return { successful: false, reason: 'speed must be positive' };
        }
        return { successful: true, reason: '' };
      });

      const result = node.setParameters({ speed: -1.0 });
      expect(result.successful).toBe(false);
      expect(result.reason).toBe('speed must be positive');
      // Value should NOT have changed
      expect(node.getParameter('speed')).toBe(1.0);
    });

    it('chains multiple callbacks - all must accept', () => {
      node.addOnSetParametersCallback((params) => {
        if ('speed' in params && params.speed < 0) {
          return { successful: false, reason: 'speed must be non-negative' };
        }
        return { successful: true, reason: '' };
      });

      node.addOnSetParametersCallback((params) => {
        if ('speed' in params && params.speed > 100) {
          return { successful: false, reason: 'speed must be <= 100' };
        }
        return { successful: true, reason: '' };
      });

      // Both pass
      expect(node.setParameters({ speed: 50 }).successful).toBe(true);

      // First rejects
      expect(node.setParameters({ speed: -5 }).successful).toBe(false);

      // Second rejects
      expect(node.setParameters({ speed: 200 }).successful).toBe(false);
    });

    it('sets multiple parameters atomically', () => {
      const result = node.setParameters({ speed: 3.0, name: 'fast' });
      expect(result.successful).toBe(true);
      expect(node.getParameter('speed')).toBe(3.0);
      expect(node.getParameter('name')).toBe('fast');
    });

    it('rejects atomically - no partial changes', () => {
      node.addOnSetParametersCallback((params) => {
        if ('speed' in params && params.speed < 0) {
          return { successful: false, reason: 'invalid speed' };
        }
        return { successful: true, reason: '' };
      });

      const result = node.setParameters({ speed: -1, name: 'should_not_change' });
      expect(result.successful).toBe(false);
      expect(node.getParameter('name')).toBe('default'); // unchanged
    });

    it('calls onParameterChange for each param after validation', () => {
      const changes = [];
      node.onParameterChange = (name, value) => {
        changes.push({ name, value });
      };

      node.setParameters({ speed: 2.0, enabled: false });
      expect(changes).toEqual([
        { name: 'speed', value: 2.0 },
        { name: 'enabled', value: false },
      ]);
    });

    it('does not call onParameterChange on rejection', () => {
      const changes = [];
      node.onParameterChange = (name, value) => {
        changes.push({ name, value });
      };

      node.addOnSetParametersCallback(() => ({
        successful: false,
        reason: 'nope',
      }));

      node.setParameters({ speed: 999 });
      expect(changes).toEqual([]);
    });
  });

  describe('setParameter (single)', () => {
    it('returns true on success', () => {
      expect(node.setParameter('speed', 2.0)).toBe(true);
      expect(node.getParameter('speed')).toBe(2.0);
    });

    it('returns false on rejection', () => {
      node.addOnSetParametersCallback(() => ({
        successful: false,
        reason: 'rejected',
      }));
      expect(node.setParameter('speed', 2.0)).toBe(false);
      expect(node.getParameter('speed')).toBe(1.0);
    });

    it('returns false for undeclared parameter', () => {
      expect(node.setParameter('nonexistent', 42)).toBe(false);
    });
  });

  describe('removeParameterCallback', () => {
    it('removes a callback by ID', () => {
      const id = node.addOnSetParametersCallback(() => ({
        successful: false,
        reason: 'blocked',
      }));

      // Should be blocked
      expect(node.setParameter('speed', 2.0)).toBe(false);

      // Remove callback
      node.removeParameterCallback(id);

      // Should now succeed
      expect(node.setParameter('speed', 2.0)).toBe(true);
    });
  });

  describe('callback error handling', () => {
    it('catches exceptions from callbacks', () => {
      node.addOnSetParametersCallback(() => {
        throw new Error('callback crashed');
      });

      const result = node.setParameters({ speed: 2.0 });
      expect(result.successful).toBe(false);
      expect(result.reason).toContain('Callback error');
      expect(node.getParameter('speed')).toBe(1.0);
    });
  });
});
