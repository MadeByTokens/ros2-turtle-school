import { describe, it, expect, beforeEach, vi } from 'vitest';
import { LocalComm } from './LocalComm.js';

describe('LocalComm', () => {
  let comm;

  beforeEach(() => {
    comm = new LocalComm();
  });

  describe('subscribe / unsubscribe', () => {
    it('returns a subscription ID', () => {
      const subId = comm.subscribe('/test', 'std_msgs/msg/String', () => {});
      expect(subId).toBeDefined();
      expect(typeof subId).toBe('string');
    });

    it('returns unique IDs for multiple subscriptions', () => {
      const id1 = comm.subscribe('/test', 'std_msgs/msg/String', () => {});
      const id2 = comm.subscribe('/test', 'std_msgs/msg/String', () => {});
      expect(id1).not.toBe(id2);
    });

    it('unsubscribes successfully', () => {
      const subId = comm.subscribe('/test', 'std_msgs/msg/String', () => {});
      expect(comm.unsubscribe(subId)).toBe(true);
    });

    it('returns false when unsubscribing non-existent ID', () => {
      expect(comm.unsubscribe('non_existent_id')).toBe(false);
    });

    it('removes callback on unsubscribe', () => {
      const callback = vi.fn();
      const subId = comm.subscribe('/test', 'std_msgs/msg/String', callback);
      comm.advertise('/test', 'std_msgs/msg/String', 'node1');

      comm.publish('/test', 'std_msgs/msg/String', { data: 'before' });
      expect(callback).toHaveBeenCalledTimes(1);

      comm.unsubscribe(subId);
      comm.publish('/test', 'std_msgs/msg/String', { data: 'after' });
      expect(callback).toHaveBeenCalledTimes(1); // Still 1, not called again
    });
  });

  describe('publish', () => {
    it('delivers messages to subscribers', () => {
      const callback = vi.fn();
      comm.subscribe('/test', 'std_msgs/msg/String', callback);
      comm.advertise('/test', 'std_msgs/msg/String', 'node1');

      comm.publish('/test', 'std_msgs/msg/String', { data: 'hello' });

      expect(callback).toHaveBeenCalledTimes(1);
      expect(callback).toHaveBeenCalledWith(expect.objectContaining({ data: 'hello' }));
    });

    it('adds timestamp to messages', () => {
      const callback = vi.fn();
      comm.subscribe('/test', 'std_msgs/msg/String', callback);
      comm.advertise('/test', 'std_msgs/msg/String', 'node1');

      comm.publish('/test', 'std_msgs/msg/String', { data: 'hello' });

      expect(callback).toHaveBeenCalledWith(expect.objectContaining({
        _timestamp: expect.any(Number)
      }));
    });

    it('preserves existing timestamp', () => {
      const callback = vi.fn();
      comm.subscribe('/test', 'std_msgs/msg/String', callback);
      comm.advertise('/test', 'std_msgs/msg/String', 'node1');

      const originalTs = 1234567890;
      comm.publish('/test', 'std_msgs/msg/String', { data: 'hello', _timestamp: originalTs });

      expect(callback).toHaveBeenCalledWith(expect.objectContaining({
        _timestamp: originalTs
      }));
    });

    it('delivers to multiple subscribers', () => {
      const callback1 = vi.fn();
      const callback2 = vi.fn();
      comm.subscribe('/test', 'std_msgs/msg/String', callback1);
      comm.subscribe('/test', 'std_msgs/msg/String', callback2);
      comm.advertise('/test', 'std_msgs/msg/String', 'node1');

      comm.publish('/test', 'std_msgs/msg/String', { data: 'hello' });

      expect(callback1).toHaveBeenCalledTimes(1);
      expect(callback2).toHaveBeenCalledTimes(1);
    });

    it('does nothing for non-existent topic', () => {
      // Should not throw
      expect(() => comm.publish('/nonexistent', 'type', { data: 'test' })).not.toThrow();
    });

    it('handles callback errors gracefully', () => {
      const errorCallback = vi.fn(() => { throw new Error('callback error'); });
      const goodCallback = vi.fn();
      comm.subscribe('/test', 'std_msgs/msg/String', errorCallback);
      comm.subscribe('/test', 'std_msgs/msg/String', goodCallback);
      comm.advertise('/test', 'std_msgs/msg/String', 'node1');

      // Should not throw, and should continue to other callbacks
      expect(() => comm.publish('/test', 'std_msgs/msg/String', { data: 'test' })).not.toThrow();
      expect(goodCallback).toHaveBeenCalled();
    });
  });

  describe('advertise / unadvertise', () => {
    it('returns a publisher ID', () => {
      const pubId = comm.advertise('/test', 'std_msgs/msg/String', 'node1');
      expect(pubId).toBeDefined();
      expect(typeof pubId).toBe('string');
    });

    it('returns unique IDs for multiple publishers', () => {
      const id1 = comm.advertise('/test', 'std_msgs/msg/String', 'node1');
      const id2 = comm.advertise('/test', 'std_msgs/msg/String', 'node2');
      expect(id1).not.toBe(id2);
    });

    it('unadvertises successfully', () => {
      const pubId = comm.advertise('/test', 'std_msgs/msg/String', 'node1');
      expect(comm.unadvertise(pubId)).toBe(true);
    });

    it('returns false when unadvertising non-existent ID', () => {
      expect(comm.unadvertise('non_existent_id')).toBe(false);
    });
  });

  describe('advertiseService / callService / unadvertiseService', () => {
    it('registers a service', () => {
      const handler = vi.fn().mockResolvedValue({ success: true });
      const serviceId = comm.advertiseService('/test_srv', 'test/srv/Test', handler, 'node1');
      expect(serviceId).toBeDefined();
    });

    it('calls service handler and returns response', async () => {
      const handler = vi.fn().mockResolvedValue({ result: 42 });
      comm.advertiseService('/test_srv', 'test/srv/Test', handler, 'node1');

      const response = await comm.callService('/test_srv', 'test/srv/Test', { input: 10 });

      expect(handler).toHaveBeenCalledWith({ input: 10 });
      expect(response).toEqual({ result: 42 });
    });

    it('throws error for non-existent service', async () => {
      await expect(comm.callService('/nonexistent', 'type', {}))
        .rejects.toThrow("Service '/nonexistent' not available");
    });

    it('throws error for type mismatch', async () => {
      comm.advertiseService('/test_srv', 'correct/srv/Type', vi.fn(), 'node1');

      await expect(comm.callService('/test_srv', 'wrong/srv/Type', {}))
        .rejects.toThrow('Service type mismatch');
    });

    it('wraps handler errors', async () => {
      const handler = vi.fn().mockRejectedValue(new Error('handler failed'));
      comm.advertiseService('/test_srv', 'test/srv/Test', handler, 'node1');

      await expect(comm.callService('/test_srv', 'test/srv/Test', {}))
        .rejects.toThrow('Service call failed: handler failed');
    });

    it('unadvertises service successfully', () => {
      const serviceId = comm.advertiseService('/test_srv', 'test/srv/Test', vi.fn(), 'node1');
      expect(comm.unadvertiseService(serviceId)).toBe(true);
    });

    it('returns false when unadvertising non-existent service', () => {
      expect(comm.unadvertiseService('non_existent_id')).toBe(false);
    });
  });

  describe('advertiseAction / sendActionGoal / cancelActionGoal', () => {
    it('registers an action server', () => {
      const handlers = {
        executeCallback: vi.fn().mockResolvedValue({ success: true }),
        goalCallback: vi.fn().mockResolvedValue(true),
        cancelCallback: vi.fn()
      };
      const actionId = comm.advertiseAction('/test_action', 'test/action/Test', handlers, 'node1');
      expect(actionId).toBeDefined();
    });

    it('executes action goal and returns result', async () => {
      const handlers = {
        executeCallback: vi.fn().mockResolvedValue({ success: true, value: 42 }),
        goalCallback: vi.fn().mockResolvedValue(true)
      };
      comm.advertiseAction('/test_action', 'test/action/Test', handlers, 'node1');

      const result = await comm.sendActionGoal('/test_action', 'test/action/Test', { target: 10 });

      expect(handlers.goalCallback).toHaveBeenCalledWith({ target: 10 });
      expect(result).toEqual({ success: true, value: 42 });
    });

    it('rejects when goal is not accepted', async () => {
      const handlers = {
        executeCallback: vi.fn().mockResolvedValue({}),
        goalCallback: vi.fn().mockResolvedValue(false)
      };
      comm.advertiseAction('/test_action', 'test/action/Test', handlers, 'node1');

      await expect(comm.sendActionGoal('/test_action', 'test/action/Test', {}))
        .rejects.toThrow('Goal rejected');
    });

    it('throws error for non-existent action', async () => {
      await expect(comm.sendActionGoal('/nonexistent', 'type', {}))
        .rejects.toThrow("Action '/nonexistent' not available");
    });

    it('throws error for type mismatch', async () => {
      const handlers = { executeCallback: vi.fn(), goalCallback: vi.fn().mockResolvedValue(true) };
      comm.advertiseAction('/test_action', 'correct/action/Type', handlers, 'node1');

      await expect(comm.sendActionGoal('/test_action', 'wrong/action/Type', {}))
        .rejects.toThrow('Action type mismatch');
    });

    it('delivers feedback during execution', async () => {
      const feedbackCallback = vi.fn();
      const handlers = {
        executeCallback: vi.fn(async (goal, goalId, sendFeedback) => {
          sendFeedback({ progress: 50 });
          sendFeedback({ progress: 100 });
          return { done: true };
        }),
        goalCallback: vi.fn().mockResolvedValue(true)
      };
      comm.advertiseAction('/test_action', 'test/action/Test', handlers, 'node1');

      await comm.sendActionGoal('/test_action', 'test/action/Test', {}, feedbackCallback);

      expect(feedbackCallback).toHaveBeenCalledTimes(2);
      expect(feedbackCallback).toHaveBeenCalledWith({ progress: 50 });
      expect(feedbackCallback).toHaveBeenCalledWith({ progress: 100 });
    });

    it('cancels action goal', async () => {
      let resolveWait;
      const waitPromise = new Promise(r => { resolveWait = r; });
      let checkCanceled;

      const handlers = {
        executeCallback: vi.fn(async (goal, goalId, sendFeedback, isCanceled) => {
          checkCanceled = isCanceled;
          // Wait until we signal to continue
          await waitPromise;
          return { done: true };
        }),
        goalCallback: vi.fn().mockResolvedValue(true),
        cancelCallback: vi.fn()
      };
      comm.advertiseAction('/test_action', 'test/action/Test', handlers, 'node1');

      // Start the action
      const goalPromise = comm.sendActionGoal('/test_action', 'test/action/Test', {});

      // Wait a tick for the goal to be registered
      await new Promise(resolve => setTimeout(resolve, 0));

      // Cancel it while it's waiting
      const action = comm.actions.get('/test_action');
      const goalId = Array.from(action.activeGoals.keys())[0];
      comm.cancelActionGoal('/test_action', goalId);

      // Now let the execution continue
      resolveWait();
      await goalPromise;

      expect(checkCanceled()).toBe(true);
      expect(handlers.cancelCallback).toHaveBeenCalledWith(goalId);
    });

    it('returns false when canceling non-existent action', () => {
      expect(comm.cancelActionGoal('/nonexistent', 'goal_id')).toBe(false);
    });

    it('returns false when canceling non-existent goal', () => {
      const handlers = { executeCallback: vi.fn() };
      comm.advertiseAction('/test_action', 'test/action/Test', handlers, 'node1');
      expect(comm.cancelActionGoal('/test_action', 'nonexistent_goal')).toBe(false);
    });

    it('unadvertises action and removes it from registry', async () => {
      const cancelCallback = vi.fn();
      let resolveWait;
      const waitPromise = new Promise(r => { resolveWait = r; });

      const handlers = {
        executeCallback: vi.fn(async (goal, goalId, sendFeedback, isCanceled) => {
          await waitPromise;
          return {};
        }),
        goalCallback: vi.fn().mockResolvedValue(true),
        cancelCallback
      };
      const actionId = comm.advertiseAction('/test_action', 'test/action/Test', handlers, 'node1');

      // Start goal execution
      const goalPromise = comm.sendActionGoal('/test_action', 'test/action/Test', {});

      // Wait a tick for the goal to be registered
      await new Promise(resolve => setTimeout(resolve, 0));

      // Verify action exists and has active goal
      expect(comm.actions.has('/test_action')).toBe(true);
      const action = comm.actions.get('/test_action');
      expect(action.activeGoals.size).toBe(1);

      // Unadvertise while goal is active
      expect(comm.unadvertiseAction(actionId)).toBe(true);
      expect(comm.actions.has('/test_action')).toBe(false);

      // Let execution continue
      resolveWait();
      await goalPromise;
    });

    it('returns false when unadvertising non-existent action', () => {
      expect(comm.unadvertiseAction('non_existent_id')).toBe(false);
    });
  });

  describe('getTopics', () => {
    it('returns empty array when no topics', () => {
      expect(comm.getTopics()).toEqual([]);
    });

    it('returns topic information', () => {
      comm.subscribe('/topic1', 'std_msgs/msg/String', () => {}, 'sub_node');
      comm.advertise('/topic1', 'std_msgs/msg/String', 'pub_node');
      comm.advertise('/topic2', 'geometry_msgs/msg/Twist', 'pub_node2');

      const topics = comm.getTopics();

      expect(topics).toHaveLength(2);

      const topic1 = topics.find(t => t.name === '/topic1');
      expect(topic1).toBeDefined();
      expect(topic1.type).toBe('std_msgs/msg/String');
      expect(topic1.publishers).toHaveLength(1);
      expect(topic1.subscribers).toHaveLength(1);
    });
  });

  describe('getServices', () => {
    it('returns empty array when no services', () => {
      expect(comm.getServices()).toEqual([]);
    });

    it('returns service information', () => {
      comm.advertiseService('/srv1', 'test/srv/Test1', vi.fn(), 'node1');
      comm.advertiseService('/srv2', 'test/srv/Test2', vi.fn(), 'node2');

      const services = comm.getServices();

      expect(services).toHaveLength(2);
      expect(services).toContainEqual(expect.objectContaining({
        name: '/srv1',
        type: 'test/srv/Test1',
        node: 'node1'
      }));
    });
  });

  describe('getActions', () => {
    it('returns empty array when no actions', () => {
      expect(comm.getActions()).toEqual([]);
    });

    it('returns action information', () => {
      const handlers = { executeCallback: vi.fn() };
      comm.advertiseAction('/action1', 'test/action/Test1', handlers, 'node1');
      comm.advertiseAction('/action2', 'test/action/Test2', handlers, 'node2');

      const actions = comm.getActions();

      expect(actions).toHaveLength(2);
      expect(actions).toContainEqual(expect.objectContaining({
        name: '/action1',
        type: 'test/action/Test1',
        node: 'node1'
      }));
    });
  });

  describe('destroy', () => {
    it('clears all topics, services, and actions', () => {
      comm.subscribe('/topic', 'type', () => {});
      comm.advertise('/topic', 'type', 'node');
      comm.advertiseService('/srv', 'type', vi.fn(), 'node');
      comm.advertiseAction('/action', 'type', { executeCallback: vi.fn() }, 'node');

      comm.destroy();

      expect(comm.getTopics()).toEqual([]);
      expect(comm.getServices()).toEqual([]);
      expect(comm.getActions()).toEqual([]);
    });

    it('marks active action goals as canceled', async () => {
      let resolveWait;
      const waitPromise = new Promise(r => { resolveWait = r; });
      let wasCanceled = false;

      const handlers = {
        executeCallback: vi.fn(async (goal, goalId, sendFeedback, isCanceled) => {
          await waitPromise;
          wasCanceled = isCanceled();
          return {};
        }),
        goalCallback: vi.fn().mockResolvedValue(true)
      };
      comm.advertiseAction('/action', 'type', handlers, 'node');

      const goalPromise = comm.sendActionGoal('/action', 'type', {});

      // Wait a tick for the goal to be registered
      await new Promise(resolve => setTimeout(resolve, 0));

      comm.destroy();

      // Let execution continue
      resolveWait();
      await goalPromise;
      expect(wasCanceled).toBe(true);
    });
  });

  describe('type mismatch warnings', () => {
    it('warns when topic type mismatch (via console)', () => {
      const warnSpy = vi.spyOn(console, 'warn').mockImplementation(() => {});

      comm.subscribe('/topic', 'type_a', () => {});
      comm.advertise('/topic', 'type_b', 'node');

      expect(warnSpy).toHaveBeenCalled();
      expect(warnSpy.mock.calls[0][0]).toContain('same type');

      warnSpy.mockRestore();
    });

    it('does not warn for unknown type', () => {
      const warnSpy = vi.spyOn(console, 'warn').mockImplementation(() => {});

      comm.subscribe('/topic', 'unknown', () => {});
      comm.advertise('/topic', 'std_msgs/msg/String', 'node');

      expect(warnSpy).not.toHaveBeenCalled();

      warnSpy.mockRestore();
    });
  });
});
