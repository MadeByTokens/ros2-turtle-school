import { describe, it, expect } from 'vitest';
import {
  DEFAULT_QOS,
  QOS_PRESETS,
  normalizeQoS,
  formatQoS,
  parseQoSArgs,
} from './qos.js';

describe('QoS utilities', () => {
  describe('DEFAULT_QOS', () => {
    it('has correct default values', () => {
      expect(DEFAULT_QOS).toEqual({
        reliability: 'reliable',
        durability: 'volatile',
        history: 'keep_last',
        depth: 10,
      });
    });

    it('is frozen', () => {
      expect(Object.isFrozen(DEFAULT_QOS)).toBe(true);
    });
  });

  describe('QOS_PRESETS', () => {
    it('has sensor_data preset with best_effort and depth 5', () => {
      expect(QOS_PRESETS.sensor_data).toEqual({
        reliability: 'best_effort',
        durability: 'volatile',
        history: 'keep_last',
        depth: 5,
      });
    });

    it('has services_default preset matching default', () => {
      expect(QOS_PRESETS.services_default.reliability).toBe('reliable');
      expect(QOS_PRESETS.services_default.depth).toBe(10);
    });

    it('has parameters preset with depth 1000', () => {
      expect(QOS_PRESETS.parameters.depth).toBe(1000);
    });

    it('has all expected preset names', () => {
      expect(Object.keys(QOS_PRESETS)).toEqual(
        expect.arrayContaining(['default', 'system_default', 'sensor_data', 'services_default', 'parameters', 'parameter_events'])
      );
    });
  });

  describe('normalizeQoS', () => {
    it('returns defaults when given undefined', () => {
      expect(normalizeQoS(undefined)).toEqual(DEFAULT_QOS);
    });

    it('returns defaults when given null', () => {
      expect(normalizeQoS(null)).toEqual(DEFAULT_QOS);
    });

    it('fills in missing fields from defaults', () => {
      const result = normalizeQoS({ reliability: 'best_effort' });
      expect(result).toEqual({
        reliability: 'best_effort',
        durability: 'volatile',
        history: 'keep_last',
        depth: 10,
      });
    });

    it('preserves all provided fields', () => {
      const qos = {
        reliability: 'best_effort',
        durability: 'transient_local',
        history: 'keep_all',
        depth: 5,
      };
      expect(normalizeQoS(qos)).toEqual(qos);
    });

    it('does not mutate the input', () => {
      const input = { reliability: 'best_effort' };
      normalizeQoS(input);
      expect(input).toEqual({ reliability: 'best_effort' });
    });
  });

  describe('formatQoS', () => {
    it('formats default QoS with correct indentation', () => {
      const lines = formatQoS(DEFAULT_QOS, '    ');
      expect(lines).toEqual([
        '    Reliability: RELIABLE',
        '    Durability: VOLATILE',
        '    History (Depth): KEEP_LAST (10)',
      ]);
    });

    it('formats best_effort QoS', () => {
      const lines = formatQoS({
        reliability: 'best_effort',
        durability: 'transient_local',
        history: 'keep_last',
        depth: 5,
      });
      expect(lines).toContain('    Reliability: BEST_EFFORT');
      expect(lines).toContain('    Durability: TRANSIENT_LOCAL');
      expect(lines).toContain('    History (Depth): KEEP_LAST (5)');
    });

    it('formats keep_all history without depth', () => {
      const lines = formatQoS({
        reliability: 'reliable',
        durability: 'volatile',
        history: 'keep_all',
        depth: 10,
      });
      expect(lines).toContain('    History (Depth): KEEP_ALL');
    });

    it('formats undefined QoS as defaults', () => {
      const lines = formatQoS(undefined);
      expect(lines).toContain('    Reliability: RELIABLE');
    });

    it('uses custom indent', () => {
      const lines = formatQoS(DEFAULT_QOS, '  ');
      expect(lines[0]).toBe('  Reliability: RELIABLE');
    });
  });

  describe('parseQoSArgs', () => {
    it('returns null qos when no QoS flags present', () => {
      const { qos, remaining } = parseQoSArgs(['echo', '/topic']);
      expect(qos).toBeNull();
      expect(remaining).toEqual(['echo', '/topic']);
    });

    it('parses --qos-reliability flag', () => {
      const { qos, remaining } = parseQoSArgs(['/topic', '--qos-reliability', 'best_effort']);
      expect(qos.reliability).toBe('best_effort');
      expect(qos.durability).toBe('volatile'); // default
      expect(remaining).toEqual(['/topic']);
    });

    it('parses --qos-durability flag', () => {
      const { qos } = parseQoSArgs(['--qos-durability', 'transient_local', '/topic']);
      expect(qos.durability).toBe('transient_local');
    });

    it('parses --qos-history flag', () => {
      const { qos } = parseQoSArgs(['--qos-history', 'keep_all', '/topic']);
      expect(qos.history).toBe('keep_all');
    });

    it('parses --qos-depth flag', () => {
      const { qos } = parseQoSArgs(['--qos-depth', '20', '/topic']);
      expect(qos.depth).toBe(20);
    });

    it('parses multiple QoS flags together', () => {
      const { qos, remaining } = parseQoSArgs([
        '/topic', '--qos-reliability', 'best_effort',
        '--qos-depth', '5', '--qos-durability', 'transient_local',
      ]);
      expect(qos).toEqual({
        reliability: 'best_effort',
        durability: 'transient_local',
        history: 'keep_last',
        depth: 5,
      });
      expect(remaining).toEqual(['/topic']);
    });

    it('parses --qos-profile preset', () => {
      const { qos } = parseQoSArgs(['--qos-profile', 'sensor_data', '/topic']);
      expect(qos.reliability).toBe('best_effort');
      expect(qos.depth).toBe(5);
    });

    it('allows individual flags to override preset', () => {
      const { qos } = parseQoSArgs([
        '--qos-profile', 'sensor_data', '--qos-depth', '20',
      ]);
      expect(qos.reliability).toBe('best_effort'); // from preset
      expect(qos.depth).toBe(20); // overridden
    });

    it('returns error for unknown preset name', () => {
      const { qos, error } = parseQoSArgs(['--qos-profile', 'nonexistent']);
      expect(qos).toBeNull();
      expect(error).toContain('Unknown QoS profile');
      expect(error).toContain('nonexistent');
    });

    it('returns error for invalid reliability value', () => {
      const { qos, error } = parseQoSArgs(['--qos-reliability', 'maybe']);
      expect(qos).toBeNull();
      expect(error).toContain('Invalid reliability');
    });

    it('returns error for invalid durability value', () => {
      const { qos, error } = parseQoSArgs(['--qos-durability', 'permanent']);
      expect(qos).toBeNull();
      expect(error).toContain('Invalid durability');
    });

    it('returns error for invalid history value', () => {
      const { qos, error } = parseQoSArgs(['--qos-history', 'forget']);
      expect(qos).toBeNull();
      expect(error).toContain('Invalid history');
    });

    it('returns error for non-numeric depth', () => {
      const { qos, error } = parseQoSArgs(['--qos-depth', 'abc']);
      expect(qos).toBeNull();
      expect(error).toContain("Invalid depth 'abc'");
    });

    it('returns error for zero depth', () => {
      const { qos, error } = parseQoSArgs(['--qos-depth', '0']);
      expect(qos).toBeNull();
      expect(error).toContain('Invalid depth');
    });

    it('returns error for negative depth', () => {
      const { qos, error } = parseQoSArgs(['--qos-depth', '-5']);
      expect(qos).toBeNull();
      expect(error).toContain('Invalid depth');
    });

    it('returns error when flag is missing its value (last arg)', () => {
      const { qos, error } = parseQoSArgs(['/topic', '--qos-depth']);
      expect(qos).toBeNull();
      expect(error).toContain('--qos-depth requires a value');
    });

    it('returns error when --qos-reliability is missing its value', () => {
      const { error } = parseQoSArgs(['--qos-reliability']);
      expect(error).toContain('--qos-reliability requires a value');
    });

    it('returns error when --qos-profile is missing its value', () => {
      const { error } = parseQoSArgs(['--qos-profile']);
      expect(error).toContain('--qos-profile requires a value');
    });

    it('strips QoS flags from remaining args', () => {
      const { remaining } = parseQoSArgs([
        '--once', '/topic', 'geometry_msgs/msg/Twist',
        '--qos-reliability', 'best_effort', '--qos-depth', '5',
      ]);
      expect(remaining).toEqual(['--once', '/topic', 'geometry_msgs/msg/Twist']);
    });
  });
});
