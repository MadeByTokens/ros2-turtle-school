import { getInterface, getDefinition, listMessages, listServices, listActions } from '../msgs/index.js';

/**
 * Handle ros2 interface commands
 */
export async function handleRos2Interface(args, terminal) {
  if (args.length === 0) {
    showHelp(terminal);
    terminal.finishCommand();
    return;
  }

  const subcommand = args[0];
  const subArgs = args.slice(1);

  switch (subcommand) {
    case 'show':
      handleShow(subArgs, terminal);
      break;

    case 'list':
      handleList(subArgs, terminal);
      break;

    case 'package':
      handlePackage(subArgs, terminal);
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
  terminal.writeln('usage: ros2 interface <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  show      Show the definition of an interface');
  terminal.writeln('  list      List all available interfaces');
  terminal.writeln('  package   List interfaces in a package');
}

/**
 * Handle ros2 interface show <type>
 */
function handleShow(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 interface show <interface_type>');
    return;
  }

  const typeName = args[0];
  const definition = getDefinition(typeName);

  if (!definition) {
    terminal.writeln(`\x1b[31mInterface '${typeName}' not found.\x1b[0m`);
    terminal.writeln('');
    terminal.writeln('Use "ros2 interface list" to see available interfaces.');
    return;
  }

  terminal.writeln(definition);
}

/**
 * Handle ros2 interface list
 */
function handleList(args, terminal) {
  const showMsgs = args.includes('-m') || args.includes('--only-msgs');
  const showSrvs = args.includes('-s') || args.includes('--only-srvs');
  const showActions = args.includes('-a') || args.includes('--only-actions');

  const showAll = !showMsgs && !showSrvs && !showActions;

  if (showAll || showMsgs) {
    terminal.writeln('Messages:');
    for (const msg of listMessages().sort()) {
      terminal.writeln(`  ${msg}`);
    }
    terminal.writeln('');
  }

  if (showAll || showSrvs) {
    terminal.writeln('Services:');
    for (const srv of listServices().sort()) {
      terminal.writeln(`  ${srv}`);
    }
    terminal.writeln('');
  }

  if (showAll || showActions) {
    terminal.writeln('Actions:');
    for (const action of listActions().sort()) {
      terminal.writeln(`  ${action}`);
    }
  }
}

/**
 * Handle ros2 interface package <package_name>
 */
function handlePackage(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 interface package <package_name>');
    return;
  }

  const packageName = args[0];
  const prefix = packageName + '/';

  const matchingMsgs = listMessages().filter(m => m.startsWith(prefix));
  const matchingSrvs = listServices().filter(s => s.startsWith(prefix));
  const matchingActions = listActions().filter(a => a.startsWith(prefix));

  if (matchingMsgs.length === 0 && matchingSrvs.length === 0 && matchingActions.length === 0) {
    terminal.writeln(`\x1b[31mNo interfaces found in package '${packageName}'.\x1b[0m`);
    return;
  }

  for (const msg of matchingMsgs.sort()) {
    terminal.writeln(msg);
  }
  for (const srv of matchingSrvs.sort()) {
    terminal.writeln(srv);
  }
  for (const action of matchingActions.sort()) {
    terminal.writeln(action);
  }
}

export default { handleRos2Interface };
