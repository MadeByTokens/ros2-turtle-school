import { SimDDS } from '../core/SimDDS.js';
import { parseMessage, formatMessage } from './messageParser.js';
import { getService, findServices } from '../msgs/index.js';
import { commandRegistry } from './commandRegistry.js';

/**
 * Handle ros2 service commands
 */
export async function handleRos2Service(args, terminal) {
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
      terminal.finishCommand();
      break;

    case 'type':
      handleType(subArgs, terminal);
      terminal.finishCommand();
      break;

    case 'info':
      handleInfo(subArgs, terminal);
      terminal.finishCommand();
      break;

    case 'find':
      handleFind(subArgs, terminal);
      terminal.finishCommand();
      break;

    case 'call':
      await handleCall(subArgs, terminal);
      terminal.finishCommand();
      break;

    case 'echo':
      handleEcho(subArgs, terminal);
      // Don't finish - runs until Ctrl+C
      break;

    default:
      terminal.writeln(`\x1b[31mUnknown subcommand: ${subcommand}\x1b[0m`);
      showHelp(terminal);
      terminal.finishCommand();
  }
}

/**
 * Show help message
 */
function showHelp(terminal) {
  terminal.writeln('usage: ros2 service <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  list    List all active services');
  terminal.writeln('  type    Show the type of a service');
  terminal.writeln('  info    Show information about a service');
  terminal.writeln('  find    Find services by type');
  terminal.writeln('  call    Call a service');
  terminal.writeln('  echo    Echo service calls');
}

/**
 * Handle ros2 service list
 */
function handleList(args, terminal) {
  const showTypes = args.includes('-t') || args.includes('--show-types');
  const services = SimDDS.getServices();

  if (services.length === 0) {
    terminal.writeln('No services are currently available.');
    return;
  }

  for (const srv of services.sort((a, b) => a.name.localeCompare(b.name))) {
    if (showTypes) {
      terminal.writeln(`${srv.name} [${srv.type}]`);
    } else {
      terminal.writeln(srv.name);
    }
  }
}

/**
 * Handle ros2 service type <service>
 */
function handleType(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 service type <service_name>');
    return;
  }

  const serviceName = args[0];
  const serviceInfo = SimDDS.getServiceInfo(serviceName);

  if (!serviceInfo) {
    terminal.writeln(`\x1b[31mService '${serviceName}' not found\x1b[0m`);
    return;
  }

  terminal.writeln(serviceInfo.type);
}

/**
 * Handle ros2 service info <service>
 */
function handleInfo(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 service info <service_name>');
    return;
  }

  const serviceName = args[0];
  const serviceInfo = SimDDS.getServiceInfo(serviceName);

  if (!serviceInfo) {
    terminal.writeln(`\x1b[31mService '${serviceName}' not found\x1b[0m`);
    return;
  }

  terminal.writeln(`Type: ${serviceInfo.type}`);
  terminal.writeln(`Node: ${serviceInfo.node}`);
}

/**
 * Handle ros2 service find <type>
 */
function handleFind(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 service find <srv_type>');
    return;
  }

  const srvType = args[0];
  const services = SimDDS.getServices();
  const matching = services.filter(s => s.type === srvType);

  if (matching.length === 0) {
    terminal.writeln(`No services of type '${srvType}' found.`);
    return;
  }

  for (const srv of matching) {
    terminal.writeln(srv.name);
  }
}

/**
 * Handle ros2 service call <service> <type> "<data>"
 */
async function handleCall(args, terminal) {
  // Parse arguments
  let serviceName = null;
  let srvType = null;
  let reqData = null;

  for (let i = 0; i < args.length; i++) {
    const arg = args[i];

    if (!serviceName) {
      serviceName = arg;
    } else if (!srvType) {
      srvType = arg;
    } else if (!reqData) {
      reqData = arg;
    }
  }

  if (!serviceName || !srvType) {
    terminal.writeln('usage: ros2 service call <service_name> <srv_type> "<request_data>"');
    return;
  }

  // Validate service type
  const srvDef = getService(srvType);
  if (!srvDef) {
    terminal.writeln(`\x1b[31mUnknown service type: ${srvType}\x1b[0m`);
    terminal.writeln('');
    terminal.writeln('Available service types:');
    const matches = findServices(srvType.split('/').pop());
    for (const match of matches.slice(0, 10)) {
      terminal.writeln(`  ${match}`);
    }
    return;
  }

  // Check if service exists
  const serviceInfo = SimDDS.getServiceInfo(serviceName);
  if (!serviceInfo) {
    terminal.writeln(`\x1b[31mService '${serviceName}' not available\x1b[0m`);
    return;
  }

  // Parse request data
  const data = reqData ? parseMessage(reqData) : {};
  const request = srvDef.createRequest(data);

  terminal.writeln(`requester: making request: ${serviceName}`);
  terminal.writeln('');

  try {
    const response = await SimDDS.callService(serviceName, srvType, request);
    terminal.writeln('response:');
    terminal.writeln(formatMessage(response));
  } catch (err) {
    terminal.writeln(`\x1b[31mService call failed: ${err.message}\x1b[0m`);
  }
}

/**
 * Handle ros2 service echo <service>
 */
function handleEcho(args, terminal) {
  // Service echo is not fully implemented in this simulation
  // Would require intercepting service calls
  terminal.writeln('\x1b[33mService echo is not fully supported in this simulation.\x1b[0m');
  terminal.finishCommand();
}

// Self-register with the command registry
commandRegistry.registerRos2('service', handleRos2Service);

export default { handleRos2Service };
