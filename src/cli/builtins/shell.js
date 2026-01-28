/**
 * Shell built-in commands - clear, ls, pwd, echo
 */

import { commandRegistry } from '../commandRegistry.js';

/**
 * Clear terminal
 */
function handleClear(args, terminal) {
  terminal.clear();
  terminal.finishCommand();
  return true;
}

/**
 * List directory (simulated)
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
  return true;
}

/**
 * Print working directory
 */
function handlePwd(args, terminal) {
  terminal.writeln('/home/ros2/ws');
  terminal.finishCommand();
  return true;
}

/**
 * Echo - print text
 */
function handleEcho(args, terminal) {
  if (args.length > 0) {
    terminal.writeln(args.join(' '));
  }
  terminal.finishCommand();
  return true;
}

// Register all shell commands
commandRegistry.register('clear', handleClear);
commandRegistry.register('ls', handleLs);
commandRegistry.register('pwd', handlePwd);
commandRegistry.register('echo', handleEcho);
