import { SimDDS } from '../core/SimDDS.js';
import { ProcessManager } from '../core/ProcessManager.js';
import { commandRegistry } from './commandRegistry.js';

/**
 * Handle ros2 node commands
 */
export async function handleRos2Node(args, terminal) {
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

    case 'info':
      handleInfo(subArgs, terminal);
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
  terminal.writeln('usage: ros2 node <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  list    List running nodes');
  terminal.writeln('  info    Show information about a node');
}

/**
 * Handle ros2 node list
 */
function handleList(args, terminal) {
  const nodes = SimDDS.getNodes();

  if (nodes.length === 0) {
    terminal.writeln('No nodes are currently running.');
    return;
  }

  for (const node of nodes.sort()) {
    terminal.writeln(node);
  }
}

/**
 * Handle ros2 node info <node_name>
 */
function handleInfo(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 node info <node_name>');
    return;
  }

  let nodeName = args[0];
  // Add leading slash if not present
  if (!nodeName.startsWith('/')) {
    nodeName = '/' + nodeName;
  }

  const nodeInfo = SimDDS.getNodeInfo(nodeName);
  if (!nodeInfo) {
    terminal.writeln(`\x1b[31mNode '${nodeName}' not found\x1b[0m`);
    return;
  }

  terminal.writeln(`${nodeName}`);

  // Get process info for more details
  const procInfo = ProcessManager.getProcessByNodeName(nodeName);
  const node = procInfo?.node;

  // Subscribers
  terminal.writeln('  Subscribers:');
  if (node && node.subscriptions.length > 0) {
    for (const sub of node.subscriptions) {
      terminal.writeln(`    ${sub.topic}: ${sub.msgType}`);
    }
  } else {
    terminal.writeln('    (none)');
  }

  // Publishers
  terminal.writeln('  Publishers:');
  if (node && node.publishers.length > 0) {
    for (const pub of node.publishers) {
      terminal.writeln(`    ${pub.topic}: ${pub.msgType}`);
    }
  } else {
    terminal.writeln('    (none)');
  }

  // Service Servers
  terminal.writeln('  Service Servers:');
  if (node && node.services.length > 0) {
    for (const srv of node.services) {
      terminal.writeln(`    ${srv.name}: ${srv.type}`);
    }
  } else {
    terminal.writeln('    (none)');
  }

  // Service Clients
  terminal.writeln('  Service Clients:');
  terminal.writeln('    (none)');

  // Action Servers
  terminal.writeln('  Action Servers:');
  if (node && node.actionServers.length > 0) {
    for (const action of node.actionServers) {
      terminal.writeln(`    ${action.name}: ${action.type}`);
    }
  } else {
    terminal.writeln('    (none)');
  }

  // Action Clients
  terminal.writeln('  Action Clients:');
  terminal.writeln('    (none)');
}

// Self-register with the command registry
commandRegistry.registerRos2('node', handleRos2Node);

export default { handleRos2Node };
