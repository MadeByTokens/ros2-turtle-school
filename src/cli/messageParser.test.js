import { describe, it, expect } from 'vitest';
import { parseMessage, formatMessage, formatMessageCompact } from './messageParser.js';

describe('parseMessage', () => {
  describe('basic key-value parsing', () => {
    it('parses simple key-value pairs', () => {
      expect(parseMessage('{x: 1.0}')).toEqual({ x: 1 });
      expect(parseMessage('{x: 1.0, y: 2.0}')).toEqual({ x: 1, y: 2 });
    });

    it('handles integers and floats', () => {
      expect(parseMessage('{count: 42}')).toEqual({ count: 42 });
      expect(parseMessage('{value: 3.14159}')).toEqual({ value: 3.14159 });
      expect(parseMessage('{neg: -5}')).toEqual({ neg: -5 });
    });

    it('handles booleans', () => {
      expect(parseMessage('{enabled: true}')).toEqual({ enabled: true });
      expect(parseMessage('{enabled: false}')).toEqual({ enabled: false });
    });

    it('handles null values', () => {
      expect(parseMessage('{data: null}')).toEqual({ data: null });
      expect(parseMessage('{data: ~}')).toEqual({ data: null });
    });
  });

  describe('nested objects', () => {
    it('parses nested objects', () => {
      expect(parseMessage('{linear: {x: 1.0}}')).toEqual({ linear: { x: 1 } });
    });

    it('parses deeply nested objects', () => {
      const result = parseMessage('{a: {b: {c: 1}}}');
      expect(result).toEqual({ a: { b: { c: 1 } } });
    });

    it('parses ROS2 Twist-like messages', () => {
      const result = parseMessage('{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}');
      expect(result).toEqual({
        linear: { x: 2, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 1 }
      });
    });
  });

  describe('arrays', () => {
    it('parses simple arrays', () => {
      expect(parseMessage('{data: [1, 2, 3]}')).toEqual({ data: [1, 2, 3] });
    });

    it('parses arrays with mixed types', () => {
      expect(parseMessage('{data: [1, true, "hello"]}')).toEqual({ data: [1, true, 'hello'] });
    });

    it('parses nested arrays', () => {
      expect(parseMessage('{matrix: [[1, 2], [3, 4]]}')).toEqual({ matrix: [[1, 2], [3, 4]] });
    });

    it('parses arrays of objects', () => {
      expect(parseMessage('{points: [{x: 1}, {x: 2}]}')).toEqual({ points: [{ x: 1 }, { x: 2 }] });
    });

    it('parses empty arrays', () => {
      expect(parseMessage('{data: []}')).toEqual({ data: [] });
    });
  });

  describe('strings', () => {
    it('parses double-quoted strings', () => {
      expect(parseMessage('{name: "turtle1"}')).toEqual({ name: 'turtle1' });
    });

    it('parses single-quoted strings', () => {
      expect(parseMessage("{name: 'turtle1'}")).toEqual({ name: 'turtle1' });
    });

    it('parses unquoted strings as strings', () => {
      expect(parseMessage('{name: turtle1}')).toEqual({ name: 'turtle1' });
    });

    it('handles strings with spaces', () => {
      expect(parseMessage('{msg: "hello world"}')).toEqual({ msg: 'hello world' });
    });
  });

  describe('edge cases', () => {
    it('handles empty input', () => {
      expect(parseMessage('')).toEqual({});
      expect(parseMessage(null)).toEqual({});
      expect(parseMessage(undefined)).toEqual({});
    });

    it('handles empty braces', () => {
      expect(parseMessage('{}')).toEqual({});
    });

    it('handles whitespace', () => {
      expect(parseMessage('  {  x:  1  }  ')).toEqual({ x: 1 });
    });

    it('handles input without braces', () => {
      expect(parseMessage('x: 1')).toEqual({ x: 1 });
    });
  });

  describe('JSON fallback', () => {
    it('handles proper JSON', () => {
      expect(parseMessage('{"x": 1, "y": 2}')).toEqual({ x: 1, y: 2 });
    });

    it('handles JSON with nested objects', () => {
      expect(parseMessage('{"linear": {"x": 1.0}}')).toEqual({ linear: { x: 1 } });
    });
  });
});

describe('formatMessage', () => {
  it('formats null values', () => {
    expect(formatMessage(null)).toBe('null');
    expect(formatMessage(undefined)).toBe('null');
  });

  it('formats primitives', () => {
    expect(formatMessage(42)).toBe('42');
    expect(formatMessage('hello')).toBe('hello');
    expect(formatMessage(true)).toBe('true');
  });

  it('formats arrays', () => {
    expect(formatMessage([])).toBe('[]');
    expect(formatMessage([1, 2, 3])).toBe('[1, 2, 3]');
  });

  it('formats nested objects with indentation', () => {
    const msg = { linear: { x: 1, y: 0 } };
    const formatted = formatMessage(msg);
    expect(formatted).toContain('linear:');
    expect(formatted).toContain('x: 1');
    expect(formatted).toContain('y: 0');
  });

  it('skips internal properties (underscore prefix)', () => {
    const msg = { x: 1, _timestamp: 123456 };
    const formatted = formatMessage(msg);
    expect(formatted).toContain('x: 1');
    expect(formatted).not.toContain('_timestamp');
  });
});

describe('formatMessageCompact', () => {
  it('formats null values', () => {
    expect(formatMessageCompact(null)).toBe('null');
    expect(formatMessageCompact(undefined)).toBe('null');
  });

  it('formats primitives', () => {
    expect(formatMessageCompact(42)).toBe('42');
    expect(formatMessageCompact('hello')).toBe('hello');
  });

  it('formats arrays on single line', () => {
    expect(formatMessageCompact([1, 2, 3])).toBe('[1, 2, 3]');
  });

  it('formats objects on single line', () => {
    expect(formatMessageCompact({ x: 1, y: 2 })).toBe('{x: 1, y: 2}');
  });

  it('formats nested objects on single line', () => {
    const msg = { linear: { x: 1 }, angular: { z: 0.5 } };
    const formatted = formatMessageCompact(msg);
    expect(formatted).toBe('{linear: {x: 1}, angular: {z: 0.5}}');
  });

  it('skips internal properties', () => {
    const msg = { x: 1, _timestamp: 123456 };
    expect(formatMessageCompact(msg)).toBe('{x: 1}');
  });
});
