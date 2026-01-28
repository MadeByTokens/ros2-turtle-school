import { handleRos2Run } from './ros2_run.js';
import { handleRos2Node } from './ros2_node.js';
import { handleRos2Topic } from './ros2_topic.js';
import { handleRos2Service } from './ros2_service.js';
import { handleRos2Action } from './ros2_action.js';
import { handleRos2Param } from './ros2_param.js';
import { handleRos2Bag } from './ros2_bag.js';
import { handleRos2Interface } from './ros2_interface.js';
import { handleRos2Pkg } from './ros2_pkg.js';

/**
 * Main command parser - routes commands to appropriate handlers
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

  // Route to appropriate handler
  switch (command) {
    case 'ros2':
      await handleRos2Command(args, terminal);
      break;

    case 'rqt':
      handleRqtCommand(args, terminal);
      break;

    case 'rqt_graph':
      handleRqtGraph(terminal);
      break;

    case 'rqt_console':
      handleRqtConsole(terminal);
      break;

    case 'clear':
      terminal.clear();
      terminal.finishCommand();
      break;

    case 'ls':
      handleLs(args, terminal);
      break;

    case 'pwd':
      terminal.writeln('/home/ros2/ws');
      terminal.finishCommand();
      break;

    case 'cd':
      terminal.writeln('cd: Simulated filesystem - directory change not supported');
      terminal.finishCommand();
      break;

    case 'cat':
    case 'echo':
      if (args.length > 0) {
        terminal.writeln(args.join(' '));
      }
      terminal.finishCommand();
      break;

    case 'whoami':
      terminal.writeln('ros2');
      terminal.finishCommand();
      break;

    case 'uname':
      terminal.writeln('ROS2WebSim 1.0.0');
      terminal.finishCommand();
      break;

    case 'help':
      showHelp(terminal);
      terminal.finishCommand();
      break;

    case 'exit':
    case 'quit':
      terminal.writeln('Use Ctrl+C to stop running commands.');
      terminal.writeln('Close terminal tabs with the X button.');
      terminal.finishCommand();
      break;

    default:
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

/**
 * Handle ros2 subcommands
 */
async function handleRos2Command(args, terminal) {
  if (args.length === 0) {
    showRos2Help(terminal);
    terminal.finishCommand();
    return;
  }

  const subcommand = args[0];
  const subArgs = args.slice(1);

  switch (subcommand) {
    case 'run':
      await handleRos2Run(subArgs, terminal);
      break;

    case 'node':
      await handleRos2Node(subArgs, terminal);
      break;

    case 'topic':
      await handleRos2Topic(subArgs, terminal);
      break;

    case 'service':
      await handleRos2Service(subArgs, terminal);
      break;

    case 'action':
      await handleRos2Action(subArgs, terminal);
      break;

    case 'param':
      await handleRos2Param(subArgs, terminal);
      break;

    case 'bag':
      await handleRos2Bag(subArgs, terminal);
      break;

    case 'interface':
      await handleRos2Interface(subArgs, terminal);
      break;

    case 'pkg':
      await handleRos2Pkg(subArgs, terminal);
      break;

    default:
      terminal.writeln(`\x1b[31mUnknown ros2 command: ${subcommand}\x1b[0m`);
      showRos2Help(terminal);
      terminal.finishCommand();
  }
}

/**
 * Handle rqt command
 */
function handleRqtCommand(args, terminal) {
  // Open rqt tool selector
  const event = new CustomEvent('open-rqt-modal');
  window.dispatchEvent(event);
  terminal.finishCommand();
}

/**
 * Handle rqt_graph command
 */
function handleRqtGraph(terminal) {
  const event = new CustomEvent('toggle-graph');
  window.dispatchEvent(event);
  terminal.writeln('Opening rqt_graph...');
  terminal.finishCommand();
}

/**
 * Handle rqt_console command
 */
function handleRqtConsole(terminal) {
  const event = new CustomEvent('toggle-console');
  window.dispatchEvent(event);
  terminal.writeln('Opening rqt_console...');
  terminal.finishCommand();
}

/**
 * Handle ls command - simulated file listing
 */
function handleLs(args, terminal) {
  // Check for -la, -l, -a flags
  const showAll = args.includes('-a') || args.includes('-la') || args.includes('-al');
  const longFormat = args.includes('-l') || args.includes('-la') || args.includes('-al');

  // Simulated directory contents
  const files = [
    { name: 'src', type: 'd', size: 4096 },
    { name: 'install', type: 'd', size: 4096 },
    { name: 'build', type: 'd', size: 4096 },
    { name: 'log', type: 'd', size: 4096 },
    { name: 'package.xml', type: '-', size: 1024 },
    { name: 'CMakeLists.txt', type: '-', size: 2048 },
    { name: 'setup.py', type: '-', size: 512 }
  ];

  const hiddenFiles = [
    { name: '.', type: 'd', size: 4096 },
    { name: '..', type: 'd', size: 4096 },
    { name: '.colcon', type: 'd', size: 4096 }
  ];

  const allFiles = showAll ? [...hiddenFiles, ...files] : files;

  if (longFormat) {
    terminal.writeln('total ' + allFiles.length);
    for (const file of allFiles) {
      const perms = file.type === 'd' ? 'drwxr-xr-x' : '-rw-r--r--';
      const size = String(file.size).padStart(6);
      const date = 'Jan 27 12:00';
      const color = file.type === 'd' ? '\x1b[34m' : '';
      const reset = file.type === 'd' ? '\x1b[0m' : '';
      terminal.writeln(`${perms} 1 ros2 ros2 ${size} ${date} ${color}${file.name}${reset}`);
    }
  } else {
    // Simple listing
    const names = allFiles.map(f => {
      const color = f.type === 'd' ? '\x1b[34m' : '';
      const reset = f.type === 'd' ? '\x1b[0m' : '';
      return `${color}${f.name}${reset}`;
    });
    terminal.writeln(names.join('  '));
  }

  terminal.finishCommand();
}

/**
 * Show general help
 */
function showHelp(terminal) {
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
}

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

export default { parseCommand };
