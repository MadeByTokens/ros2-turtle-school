/**
 * QoS (Quality of Service) profile utilities for the ROS 2 emulator.
 *
 * Provides default profiles, named presets, normalization, CLI flag parsing,
 * and formatting for verbose topic info output.
 */

// ── Default profile ─────────────────────────────────────────────────
export const DEFAULT_QOS = Object.freeze({
  reliability: 'reliable',
  durability: 'volatile',
  history: 'keep_last',
  depth: 10,
});

// ── Named presets (match rclpy / rclcpp) ────────────────────────────
export const QOS_PRESETS = Object.freeze({
  default: { ...DEFAULT_QOS },
  system_default: { ...DEFAULT_QOS },
  sensor_data: {
    reliability: 'best_effort',
    durability: 'volatile',
    history: 'keep_last',
    depth: 5,
  },
  services_default: {
    reliability: 'reliable',
    durability: 'volatile',
    history: 'keep_last',
    depth: 10,
  },
  parameters: {
    reliability: 'reliable',
    durability: 'volatile',
    history: 'keep_last',
    depth: 1000,
  },
  parameter_events: {
    reliability: 'reliable',
    durability: 'volatile',
    history: 'keep_last',
    depth: 1000,
  },
});

// ── Valid policy values ─────────────────────────────────────────────
const VALID_RELIABILITY = ['reliable', 'best_effort'];
const VALID_DURABILITY = ['volatile', 'transient_local'];
const VALID_HISTORY = ['keep_last', 'keep_all'];

// ── Helpers ─────────────────────────────────────────────────────────

/**
 * Fill in any missing fields with defaults.
 * @param {Object|undefined} qos - Partial or undefined QoS object
 * @returns {Object} Complete QoS profile
 */
export function normalizeQoS(qos) {
  if (!qos) return { ...DEFAULT_QOS };
  return {
    reliability: qos.reliability || DEFAULT_QOS.reliability,
    durability: qos.durability || DEFAULT_QOS.durability,
    history: qos.history || DEFAULT_QOS.history,
    depth: qos.depth != null ? qos.depth : DEFAULT_QOS.depth,
  };
}

/**
 * Format a QoS profile for CLI verbose output (matches real ros2 CLI).
 * Returns an array of pre-indented lines.
 * @param {Object} qos - QoS profile object
 * @param {string} indent - Leading whitespace (default 4 spaces)
 * @returns {string[]}
 */
export function formatQoS(qos, indent = '    ') {
  const q = normalizeQoS(qos);
  const historyStr = q.history === 'keep_last'
    ? `KEEP_LAST (${q.depth})`
    : 'KEEP_ALL';
  return [
    `${indent}Reliability: ${q.reliability.toUpperCase()}`,
    `${indent}Durability: ${q.durability.toUpperCase()}`,
    `${indent}History (Depth): ${historyStr}`,
  ];
}

/**
 * Parse QoS-related flags from a CLI args array.
 * Supports both --qos-profile <preset> and individual flags.
 * Returns { qos, remaining } where remaining has QoS flags stripped.
 * @param {string[]} args
 * @returns {{ qos: Object|null, remaining: string[] }}
 */
export function parseQoSArgs(args) {
  const remaining = [];
  let profile = null;
  let reliability = null;
  let durability = null;
  let history = null;
  let depth = null;

  const qosFlags = ['--qos-profile', '--qos-reliability', '--qos-durability', '--qos-history', '--qos-depth'];

  for (let i = 0; i < args.length; i++) {
    const arg = args[i];
    // Check for missing value on any QoS flag
    if (qosFlags.includes(arg) && i + 1 >= args.length) {
      return { qos: null, remaining, error: `${arg} requires a value` };
    }
    if (arg === '--qos-profile') {
      profile = args[++i];
    } else if (arg === '--qos-reliability') {
      reliability = args[++i];
    } else if (arg === '--qos-durability') {
      durability = args[++i];
    } else if (arg === '--qos-history') {
      history = args[++i];
    } else if (arg === '--qos-depth') {
      const depthStr = args[++i];
      depth = parseInt(depthStr, 10);
      if (isNaN(depth) || depth <= 0) {
        return {
          qos: null,
          remaining,
          error: `Invalid depth '${depthStr}'. Must be a positive integer.`,
        };
      }
    } else {
      remaining.push(arg);
    }
  }

  // No QoS flags at all
  if (!profile && !reliability && !durability && !history && depth == null) {
    return { qos: null, remaining };
  }

  // Start from preset or defaults
  let base;
  if (profile) {
    base = QOS_PRESETS[profile];
    if (!base) {
      return {
        qos: null,
        remaining,
        error: `Unknown QoS profile '${profile}'. Available: ${Object.keys(QOS_PRESETS).join(', ')}`,
      };
    }
    base = { ...base };
  } else {
    base = { ...DEFAULT_QOS };
  }

  // Override with individual flags
  if (reliability != null) {
    if (!VALID_RELIABILITY.includes(reliability)) {
      return {
        qos: null,
        remaining,
        error: `Invalid reliability '${reliability}'. Must be: ${VALID_RELIABILITY.join(' or ')}`,
      };
    }
    base.reliability = reliability;
  }
  if (durability != null) {
    if (!VALID_DURABILITY.includes(durability)) {
      return {
        qos: null,
        remaining,
        error: `Invalid durability '${durability}'. Must be: ${VALID_DURABILITY.join(' or ')}`,
      };
    }
    base.durability = durability;
  }
  if (history != null) {
    if (!VALID_HISTORY.includes(history)) {
      return {
        qos: null,
        remaining,
        error: `Invalid history '${history}'. Must be: ${VALID_HISTORY.join(' or ')}`,
      };
    }
    base.history = history;
  }
  if (depth != null) {
    base.depth = depth;
  }

  return { qos: base, remaining };
}
