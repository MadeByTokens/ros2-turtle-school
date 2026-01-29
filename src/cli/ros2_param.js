import { SimDDS } from '../core/SimDDS.js';
import { ProcessManager } from '../core/ProcessManager.js';
import { commandRegistry } from './commandRegistry.js';

/**
 * Handle ros2 param commands
 */
export async function handleRos2Param(args, terminal) {
  if (args.length === 0) {
    showHelp(terminal);
    terminal.finishCommand();
    return;
  }

  const subcommand = args[0];
  const subArgs = args.slice(1);

  switch (subcommand) {
    case 'list':
      handleList(subArgs, terminal);
      break;

    case 'get':
      handleGet(subArgs, terminal);
      break;

    case 'set':
      handleSet(subArgs, terminal);
      break;

    case 'dump':
      handleDump(subArgs, terminal);
      break;

    case 'load':
      handleLoad(subArgs, terminal);
      break;

    default:
      terminal.writeln(`\x1b[31mUnknown subcommand: ${subcommand}\x1b[0m`);
      showHelp(terminal);
  }

  terminal.finishCommand();
}

/**
 * Show help message
 */
function showHelp(terminal) {
  terminal.writeln('usage: ros2 param <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  list    List all parameters');
  terminal.writeln('  get     Get a parameter value');
  terminal.writeln('  set     Set a parameter value');
  terminal.writeln('  dump    Dump all parameters for a node');
  terminal.writeln('  load    Load parameters from a file (not supported in sim)');
}

/**
 * Get node from args, normalizing the name
 */
function getNodeName(name) {
  if (!name) return null;
  return name.startsWith('/') ? name : '/' + name;
}

/**
 * Get node process
 */
function getNodeProcess(nodeName) {
  return ProcessManager.getProcessByNodeName(nodeName);
}

/**
 * Handle ros2 param list [node_name]
 */
function handleList(args, terminal) {
  const nodeName = args.length > 0 ? getNodeName(args[0]) : null;

  if (nodeName) {
    // List parameters for specific node
    const proc = getNodeProcess(nodeName);
    if (!proc) {
      terminal.writeln(`\x1b[31mNode '${nodeName}' not found\x1b[0m`);
      return;
    }

    const params = proc.node.getParameterNames();
    if (params.length === 0) {
      terminal.writeln('No parameters.');
      return;
    }

    terminal.writeln(`${nodeName}:`);
    for (const param of params.sort()) {
      terminal.writeln(`  ${param}`);
    }
  } else {
    // List all nodes with parameters
    const nodes = SimDDS.getNodes();

    for (const nodeId of nodes.sort()) {
      const proc = getNodeProcess(nodeId);
      if (proc && proc.node) {
        const params = proc.node.getParameterNames();
        if (params.length > 0) {
          terminal.writeln(`${nodeId}:`);
          for (const param of params.sort()) {
            terminal.writeln(`  ${param}`);
          }
        }
      }
    }
  }
}

/**
 * Handle ros2 param get <node> <param>
 */
function handleGet(args, terminal) {
  if (args.length < 2) {
    terminal.writeln('usage: ros2 param get <node_name> <parameter_name>');
    return;
  }

  const nodeName = getNodeName(args[0]);
  const paramName = args[1];

  const proc = getNodeProcess(nodeName);
  if (!proc) {
    terminal.writeln(`\x1b[31mNode '${nodeName}' not found\x1b[0m`);
    return;
  }

  const value = proc.node.getParameter(paramName);
  if (value === undefined) {
    terminal.writeln(`\x1b[31mParameter '${paramName}' not found on node '${nodeName}'\x1b[0m`);
    return;
  }

  terminal.writeln(`${getTypeLabel(value)} ${formatValue(value)}`);
}

/**
 * Handle ros2 param set <node> <param> <value>
 */
function handleSet(args, terminal) {
  if (args.length < 3) {
    terminal.writeln('usage: ros2 param set <node_name> <parameter_name> <value>');
    return;
  }

  const nodeName = getNodeName(args[0]);
  const paramName = args[1];
  const valueStr = args.slice(2).join(' ');

  const proc = getNodeProcess(nodeName);
  if (!proc) {
    terminal.writeln(`\x1b[31mNode '${nodeName}' not found\x1b[0m`);
    return;
  }

  // Parse value
  const value = parseValue(valueStr);

  // Try to set
  const success = proc.node.setParameter(paramName, value);
  if (!success) {
    terminal.writeln(`\x1b[31mParameter '${paramName}' not found on node '${nodeName}'\x1b[0m`);
    return;
  }

  terminal.writeln(`Set parameter '${paramName}' to: ${formatValue(value)}`);
}

/**
 * Handle ros2 param dump <node>
 */
function handleDump(args, terminal) {
  if (args.length < 1) {
    terminal.writeln('usage: ros2 param dump <node_name>');
    return;
  }

  const nodeName = getNodeName(args[0]);

  const proc = getNodeProcess(nodeName);
  if (!proc) {
    terminal.writeln(`\x1b[31mNode '${nodeName}' not found\x1b[0m`);
    return;
  }

  const params = proc.node.getParameterNames();
  if (params.length === 0) {
    terminal.writeln('No parameters.');
    return;
  }

  terminal.writeln(`${nodeName}:`);
  terminal.writeln('  ros__parameters:');
  for (const param of params.sort()) {
    const value = proc.node.getParameter(param);
    terminal.writeln(`    ${param}: ${formatValue(value)}`);
  }
}

/**
 * Handle ros2 param load <node> <file>
 */
function handleLoad(args, terminal) {
  terminal.writeln('\x1b[33mParameter file loading is not supported in this simulation.\x1b[0m');
  terminal.writeln('Use ros2 param set to set individual parameters.');
}

/**
 * Get the type label for a parameter value (matches ROS 2 CLI format)
 */
function getTypeLabel(value) {
  if (typeof value === 'boolean') return 'Boolean value is:';
  if (typeof value === 'number') {
    return Number.isInteger(value) ? 'Integer value is:' : 'Double value is:';
  }
  if (typeof value === 'string') return 'String value is:';
  if (Array.isArray(value)) {
    if (value.length === 0) return 'String values are:';
    const first = value[0];
    if (typeof first === 'boolean') return 'Boolean values are:';
    if (typeof first === 'number') {
      return Number.isInteger(first) ? 'Integer values are:' : 'Double values are:';
    }
    return 'String values are:';
  }
  return 'String value is:';
}

/**
 * Parse a value string
 */
function parseValue(str) {
  // Boolean
  if (str === 'true') return true;
  if (str === 'false') return false;

  // Number
  const num = Number(str);
  if (!isNaN(num)) return num;

  // String (remove quotes if present)
  if ((str.startsWith('"') && str.endsWith('"')) ||
      (str.startsWith("'") && str.endsWith("'"))) {
    return str.slice(1, -1);
  }

  return str;
}

/**
 * Format a value for display
 */
function formatValue(value) {
  if (typeof value === 'string') {
    return `"${value}"`;
  }
  if (typeof value === 'boolean') {
    return value ? 'true' : 'false';
  }
  if (typeof value === 'number') {
    return Number.isInteger(value) ? value.toString() : value.toFixed(6);
  }
  if (Array.isArray(value)) {
    return `[${value.map(formatValue).join(', ')}]`;
  }
  return String(value);
}

// Self-register with the command registry
commandRegistry.registerRos2('param', handleRos2Param);

export default { handleRos2Param };
