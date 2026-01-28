/**
 * messageParser - Parse YAML-like message strings into JavaScript objects
 * Handles ROS2 CLI message format: "{linear: {x: 2.0}, angular: {z: 1.0}}"
 */

/**
 * Parse a YAML-like message string into an object
 * @param {string} input - The message string
 * @returns {Object} Parsed object
 */
export function parseMessage(input) {
  if (!input || typeof input !== 'string') {
    return {};
  }

  // Remove outer braces if present
  let str = input.trim();
  if (str.startsWith('{') && str.endsWith('}')) {
    str = str.slice(1, -1);
  }

  // Handle empty input
  if (!str.trim()) {
    return {};
  }

  try {
    // Try JSON parse first (handles proper JSON)
    // This is the fast path - JSON.parse is highly optimized in V8
    return JSON.parse(`{${str}}`);
  } catch (e) {
    // PERFORMANCE NOTE: YAML-like parser fallback
    // The parseYamlLike() function below is a character-by-character parser
    // with O(n) time complexity. For typical ROS2 message sizes (< 1KB),
    // this is fast enough (< 1ms). For very large messages, consider:
    // 1. Using proper JSON format in commands when possible
    // 2. The parser handles nested objects/arrays but allocates intermediate strings
    // 3. No caching is implemented since messages are typically unique
    return parseYamlLike(str);
  }
}

/**
 * Parse YAML-like syntax
 * @param {string} str - The string to parse
 * @returns {Object} Parsed object
 */
function parseYamlLike(str) {
  const result = {};
  let depth = 0;
  let currentKey = '';
  let currentValue = '';
  let inValue = false;
  let inString = false;
  let stringChar = '';
  let i = 0;

  while (i < str.length) {
    const char = str[i];

    // Handle string literals
    if ((char === '"' || char === "'") && !inString) {
      inString = true;
      stringChar = char;
      if (inValue) currentValue += char;
      i++;
      continue;
    }

    if (inString && char === stringChar) {
      inString = false;
      if (inValue) currentValue += char;
      i++;
      continue;
    }

    if (inString) {
      if (inValue) currentValue += char;
      else currentKey += char;
      i++;
      continue;
    }

    // Track brace depth
    if (char === '{' || char === '[') {
      depth++;
      if (inValue) currentValue += char;
      i++;
      continue;
    }

    if (char === '}' || char === ']') {
      depth--;
      if (inValue) currentValue += char;
      i++;
      continue;
    }

    // Handle key-value separator
    if (char === ':' && depth === 0 && !inValue) {
      inValue = true;
      i++;
      // Skip whitespace after colon
      while (i < str.length && str[i] === ' ') i++;
      continue;
    }

    // Handle comma separator (at depth 0)
    if (char === ',' && depth === 0) {
      // Save current key-value pair
      if (currentKey.trim()) {
        result[currentKey.trim()] = parseValue(currentValue.trim());
      }
      currentKey = '';
      currentValue = '';
      inValue = false;
      i++;
      // Skip whitespace after comma
      while (i < str.length && str[i] === ' ') i++;
      continue;
    }

    // Accumulate characters
    if (inValue) {
      currentValue += char;
    } else {
      currentKey += char;
    }
    i++;
  }

  // Save last key-value pair
  if (currentKey.trim()) {
    result[currentKey.trim()] = parseValue(currentValue.trim());
  }

  return result;
}

/**
 * Parse a value string into appropriate type
 * @param {string} str - The value string
 * @returns {*} Parsed value
 */
function parseValue(str) {
  if (!str) return null;

  // Check for nested object
  if (str.startsWith('{') && str.endsWith('}')) {
    return parseMessage(str);
  }

  // Check for array
  if (str.startsWith('[') && str.endsWith(']')) {
    return parseArray(str.slice(1, -1));
  }

  // Check for string
  if ((str.startsWith('"') && str.endsWith('"')) ||
      (str.startsWith("'") && str.endsWith("'"))) {
    return str.slice(1, -1);
  }

  // Check for boolean
  if (str === 'true') return true;
  if (str === 'false') return false;

  // Check for null
  if (str === 'null' || str === '~') return null;

  // Check for number
  const num = Number(str);
  if (!isNaN(num)) {
    return num;
  }

  // Return as string
  return str;
}

/**
 * Parse an array string
 * @param {string} str - The array content string
 * @returns {Array} Parsed array
 */
function parseArray(str) {
  const result = [];
  let depth = 0;
  let current = '';
  let inString = false;
  let stringChar = '';

  for (let i = 0; i < str.length; i++) {
    const char = str[i];

    // Handle strings
    if ((char === '"' || char === "'") && !inString) {
      inString = true;
      stringChar = char;
      current += char;
      continue;
    }

    if (inString && char === stringChar) {
      inString = false;
      current += char;
      continue;
    }

    if (inString) {
      current += char;
      continue;
    }

    // Track depth
    if (char === '{' || char === '[') {
      depth++;
      current += char;
      continue;
    }

    if (char === '}' || char === ']') {
      depth--;
      current += char;
      continue;
    }

    // Handle comma at depth 0
    if (char === ',' && depth === 0) {
      if (current.trim()) {
        result.push(parseValue(current.trim()));
      }
      current = '';
      continue;
    }

    current += char;
  }

  // Add last element
  if (current.trim()) {
    result.push(parseValue(current.trim()));
  }

  return result;
}

/**
 * Format a message object as a string for display
 * @param {Object} msg - The message object
 * @param {number} indent - Indentation level
 * @returns {string} Formatted string
 */
export function formatMessage(msg, indent = 0) {
  if (msg === null || msg === undefined) {
    return 'null';
  }

  if (typeof msg !== 'object') {
    return String(msg);
  }

  if (Array.isArray(msg)) {
    if (msg.length === 0) return '[]';
    const items = msg.map(item => formatMessage(item, indent));
    return `[${items.join(', ')}]`;
  }

  const spaces = '  '.repeat(indent);
  const lines = [];

  for (const [key, value] of Object.entries(msg)) {
    // Skip internal properties
    if (key.startsWith('_')) continue;

    if (typeof value === 'object' && value !== null && !Array.isArray(value)) {
      lines.push(`${spaces}${key}:`);
      lines.push(formatMessage(value, indent + 1));
    } else {
      const formattedValue = formatMessage(value, indent);
      lines.push(`${spaces}${key}: ${formattedValue}`);
    }
  }

  return lines.join('\n');
}

/**
 * Format a message as compact single-line string
 * @param {Object} msg - The message object
 * @returns {string} Formatted string
 */
export function formatMessageCompact(msg) {
  if (msg === null || msg === undefined) {
    return 'null';
  }

  if (typeof msg !== 'object') {
    return String(msg);
  }

  if (Array.isArray(msg)) {
    const items = msg.map(item => formatMessageCompact(item));
    return `[${items.join(', ')}]`;
  }

  const parts = [];
  for (const [key, value] of Object.entries(msg)) {
    // Skip internal properties
    if (key.startsWith('_')) continue;
    parts.push(`${key}: ${formatMessageCompact(value)}`);
  }

  return `{${parts.join(', ')}}`;
}

export default {
  parseMessage,
  formatMessage,
  formatMessageCompact
};
