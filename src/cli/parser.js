/**
 * Command Parser - Routes all commands to handlers via the command registry.
 * No more switch statement - all commands are registered handlers.
 */

import { commandRegistry } from './commandRegistry.js';

// Import all ROS2 subcommand handlers to trigger registration
import './ros2_run.js';
import './ros2_node.js';
import './ros2_topic.js';
import './ros2_service.js';
import './ros2_action.js';
import './ros2_param.js';
import './ros2_bag.js';
import './ros2_interface.js';
import './ros2_pkg.js';

// Import all builtin commands to trigger registration
import './builtins/index.js';

/**
 * Main command parser - routes all commands through the registry
 */
export async function parseCommand(input, terminal) {
  const trimmed = input.trim();

  if (!trimmed) {
    terminal.finishCommand();
    return;
  }

  // Parse the command
  const parts = parseCommandLine(trimmed);

  if (parts.length === 0) {
    terminal.finishCommand();
    return;
  }

  const command = parts[0];
  const args = parts.slice(1);

  // Try to execute via registry
  const handled = await commandRegistry.execute(command, args, terminal);

  if (!handled) {
    terminal.writeln(`\x1b[31mCommand not found: ${command}\x1b[0m`);
    terminal.writeln('Type "help" for available commands.');
    terminal.finishCommand();
  }
}

/**
 * Parse command line into parts, handling quoted strings
 */
function parseCommandLine(input) {
  const parts = [];
  let current = '';
  let inQuotes = false;
  let quoteChar = '';
  let i = 0;

  while (i < input.length) {
    const char = input[i];

    // Handle quotes
    if ((char === '"' || char === "'") && !inQuotes) {
      inQuotes = true;
      quoteChar = char;
      i++;
      continue;
    }

    if (inQuotes && char === quoteChar) {
      inQuotes = false;
      quoteChar = '';
      i++;
      continue;
    }

    // Handle spaces outside quotes
    if (char === ' ' && !inQuotes) {
      if (current) {
        parts.push(current);
        current = '';
      }
      i++;
      continue;
    }

    current += char;
    i++;
  }

  if (current) {
    parts.push(current);
  }

  return parts;
}

export default { parseCommand };
