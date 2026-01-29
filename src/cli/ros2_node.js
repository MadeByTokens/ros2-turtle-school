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

  const sections = [
    { label: 'Subscribers', items: node?.subscriptions?.map(s => `${s.topic}: ${s.msgType}`) || [] },
    { label: 'Publishers', items: node?.publishers?.map(p => `${p.topic}: ${p.msgType}`) || [] },
    { label: 'Service Servers', items: node?.services?.map(s => `${s.name}: ${s.type}`) || [] },
    { label: 'Service Clients', items: [] },
    { label: 'Action Servers', items: node?.actionServers?.map(a => `${a.name}: ${a.type}`) || [] },
    { label: 'Action Clients', items: [] },
  ];

  for (const section of sections) {
    terminal.writeln(`  ${section.label}:`);
    if (section.items.length > 0) {
      for (const item of section.items) {
        terminal.writeln(`    ${item}`);
      }
    }
  }
}

// Self-register with the command registry
commandRegistry.registerRos2('node', handleRos2Node);

export default { handleRos2Node };
