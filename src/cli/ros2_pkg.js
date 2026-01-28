import { nodeRegistry } from '../nodes/registry.js';

/**
 * Handle ros2 pkg commands
 */
export async function handleRos2Pkg(args, terminal) {
  if (args.length === 0) {
    showHelp(terminal);
    terminal.finishCommand();
    return;
  }

  const subcommand = args[0];
  const subArgs = args.slice(1);

  switch (subcommand) {
    case 'executables':
      handleExecutables(subArgs, terminal);
      break;

    case 'list':
      handleList(subArgs, terminal);
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
  terminal.writeln('usage: ros2 pkg <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  executables   List executables in a package');
  terminal.writeln('  list          List available packages');
}

/**
 * Handle ros2 pkg executables [package_name]
 */
function handleExecutables(args, terminal) {
  if (args.length === 0) {
    // List all executables
    const packages = nodeRegistry.listPackages();

    for (const pkg of packages.sort()) {
      const execs = nodeRegistry.listExecutables(pkg);
      for (const exec of execs.sort()) {
        terminal.writeln(`${pkg} ${exec}`);
      }
    }
    return;
  }

  const packageName = args[0];
  const executables = nodeRegistry.listExecutables(packageName);

  if (executables.length === 0) {
    terminal.writeln(`\x1b[31mPackage '${packageName}' not found or has no executables.\x1b[0m`);
    return;
  }

  for (const exec of executables.sort()) {
    terminal.writeln(`${packageName} ${exec}`);
  }
}

/**
 * Handle ros2 pkg list
 */
function handleList(args, terminal) {
  const packages = nodeRegistry.listPackages();

  if (packages.length === 0) {
    terminal.writeln('No packages available.');
    return;
  }

  for (const pkg of packages.sort()) {
    terminal.writeln(pkg);
  }
}

export default { handleRos2Pkg };
