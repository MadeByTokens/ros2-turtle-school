import { nodeRegistry } from '../nodes/registry.js';
import { ProcessManager } from '../core/ProcessManager.js';
import { LogManager } from '../core/LogManager.js';

/**
 * Handle ros2 run command
 * Usage: ros2 run <package> <executable> [--ros-args ...]
 */
export async function handleRos2Run(args, terminal) {
  if (args.length < 2) {
    terminal.writeln('usage: ros2 run <package_name> <executable_name>');
    terminal.writeln('');
    terminal.writeln('Run an executable from a package.');
    terminal.finishCommand();
    return;
  }

  const packageName = args[0];
  const executableName = args[1];

  // Parse --ros-args options
  const rosArgs = parseRosArgs(args.slice(2));

  // Check if package/executable exists
  const nodeClass = nodeRegistry.getNode(packageName, executableName);
  if (!nodeClass) {
    terminal.writeln(`\x1b[31mPackage '${packageName}' not found or executable '${executableName}' not found\x1b[0m`);
    terminal.writeln('');
    terminal.writeln('Available packages and executables:');

    const packages = nodeRegistry.listPackages();
    for (const pkg of packages) {
      const execs = nodeRegistry.listExecutables(pkg);
      for (const exec of execs) {
        terminal.writeln(`  ${pkg} ${exec}`);
      }
    }

    terminal.finishCommand();
    return;
  }

  // Determine node name
  let nodeName = executableName;
  if (rosArgs.remap && rosArgs.remap['__node']) {
    nodeName = rosArgs.remap['__node'];
  }

  // Check if node already running with this name
  const fullNodeName = `/${nodeName}`;
  if (ProcessManager.isNodeRunning(fullNodeName)) {
    terminal.writeln(`\x1b[33mNode with name '${fullNodeName}' is already running\x1b[0m`);
    terminal.finishCommand();
    return;
  }

  // Set log level if specified
  if (rosArgs.logLevel) {
    LogManager.setNodeLevel(fullNodeName, rosArgs.logLevel);
  }

  try {
    // Create node instance
    const node = new nodeClass(nodeName, {
      namespace: rosArgs.namespace,
      parameters: rosArgs.params,
      remappings: rosArgs.remap
    });

    // Spawn the node
    const processId = ProcessManager.spawn(node, terminal.id, packageName, executableName);

    terminal.writeln(`\x1b[32m[INFO] Started ${packageName}/${executableName}\x1b[0m`);

    // For interactive nodes like turtlesim, don't finish - wait for Ctrl+C
    // The node runs in background via timers

  } catch (err) {
    terminal.writeln(`\x1b[31mError starting node: ${err.message}\x1b[0m`);
    terminal.finishCommand();
  }
}

/**
 * Parse --ros-args options
 */
function parseRosArgs(args) {
  const result = {
    remap: {},
    params: {},
    namespace: '',
    logLevel: null,
    paramsFile: null
  };

  let i = 0;
  let inRosArgs = false;

  while (i < args.length) {
    const arg = args[i];

    if (arg === '--ros-args') {
      inRosArgs = true;
      i++;
      continue;
    }

    if (!inRosArgs) {
      i++;
      continue;
    }

    // Handle --remap or -r
    if (arg === '--remap' || arg === '-r') {
      if (i + 1 < args.length) {
        const remapArg = args[i + 1];
        const parts = remapArg.split(':=');
        if (parts.length === 2) {
          result.remap[parts[0]] = parts[1];
        }
        i += 2;
      } else {
        i++;
      }
      continue;
    }

    // Handle remapping directly (old:=new)
    if (arg.includes(':=')) {
      const parts = arg.split(':=');
      if (parts.length === 2) {
        result.remap[parts[0]] = parts[1];
      }
      i++;
      continue;
    }

    // Handle --param or -p
    if (arg === '--param' || arg === '-p') {
      if (i + 1 < args.length) {
        const paramArg = args[i + 1];
        const parts = paramArg.split(':=');
        if (parts.length === 2) {
          result.params[parts[0]] = parseParamValue(parts[1]);
        }
        i += 2;
      } else {
        i++;
      }
      continue;
    }

    // Handle --params-file
    if (arg === '--params-file') {
      if (i + 1 < args.length) {
        result.paramsFile = args[i + 1];
        i += 2;
      } else {
        i++;
      }
      continue;
    }

    // Handle --log-level
    if (arg === '--log-level') {
      if (i + 1 < args.length) {
        result.logLevel = args[i + 1].toUpperCase();
        i += 2;
      } else {
        i++;
      }
      continue;
    }

    // Handle -n (namespace)
    if (arg === '-n' || arg === '--namespace') {
      if (i + 1 < args.length) {
        result.namespace = args[i + 1];
        i += 2;
      } else {
        i++;
      }
      continue;
    }

    i++;
  }

  return result;
}

/**
 * Parse a parameter value string
 */
function parseParamValue(str) {
  // Boolean
  if (str === 'true') return true;
  if (str === 'false') return false;

  // Number
  const num = Number(str);
  if (!isNaN(num)) return num;

  // String
  return str;
}

export default { handleRos2Run };
