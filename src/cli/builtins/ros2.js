/**
 * ROS2 main command handler
 */

import { commandRegistry } from '../commandRegistry.js';

/**
 * Show ros2 help
 */
function showRos2Help(terminal) {
  terminal.writeln('usage: ros2 <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  run        Run an executable from a package');
  terminal.writeln('  node       Node-related commands');
  terminal.writeln('  topic      Topic-related commands');
  terminal.writeln('  service    Service-related commands');
  terminal.writeln('  action     Action-related commands');
  terminal.writeln('  param      Parameter-related commands');
  terminal.writeln('  bag        Bag file commands');
  terminal.writeln('  interface  Interface introspection');
  terminal.writeln('  pkg        Package-related commands');
}

/**
 * Handle ros2 command - routes to subcommands via registry
 */
async function handleRos2(args, terminal) {
  if (args.length === 0 || args[0] === '--help' || args[0] === '-h') {
    showRos2Help(terminal);
    terminal.finishCommand();
    return true;
  }

  const subcommand = args[0];
  const subArgs = args.slice(1);

  // Try to execute via registry
  const handled = await commandRegistry.executeRos2(subcommand, subArgs, terminal);

  if (!handled) {
    terminal.writeln(`\x1b[31mUnknown ros2 command: ${subcommand}\x1b[0m`);
    showRos2Help(terminal);
    terminal.finishCommand();
  }

  return true;
}

// Register ros2 command
commandRegistry.register('ros2', handleRos2);
