/**
 * Help command - show available commands
 */

import { commandRegistry } from '../commandRegistry.js';

/**
 * Show general help
 */
function handleHelp(args, terminal) {
  terminal.writeln('ROS2 Web Emulator - Available Commands:');
  terminal.writeln('');
  terminal.writeln('\x1b[33mROS2 Commands:\x1b[0m');
  terminal.writeln('  ros2 <subcommand>  - ROS2 CLI commands (type "ros2 --help")');
  terminal.writeln('  rqt               - Open rqt tool selector');
  terminal.writeln('  rqt_graph         - Open node graph visualization');
  terminal.writeln('  rqt_console       - Open log console');
  terminal.writeln('');
  terminal.writeln('\x1b[33mShell Commands:\x1b[0m');
  terminal.writeln('  ls [-la]          - List directory contents');
  terminal.writeln('  pwd               - Print working directory');
  terminal.writeln('  clear             - Clear the terminal');
  terminal.writeln('  echo <text>       - Print text');
  terminal.writeln('  whoami            - Print current user');
  terminal.writeln('  help              - Show this help message');
  terminal.writeln('');
  terminal.writeln('Type "ros2 --help" for ros2 subcommands.');
  terminal.finishCommand();
  return true;
}

// Register help command
commandRegistry.register('help', handleHelp);
