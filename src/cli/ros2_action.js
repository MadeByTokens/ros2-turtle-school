import { SimDDS } from '../core/SimDDS.js';
import { parseMessage, formatMessage } from './messageParser.js';
import { getAction, findActions } from '../msgs/index.js';
import { commandRegistry } from './commandRegistry.js';

/**
 * Handle ros2 action commands
 */
export async function handleRos2Action(args, terminal) {
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

    case 'send_goal':
      await handleSendGoal(subArgs, terminal);
      terminal.finishCommand();
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
  terminal.writeln('usage: ros2 action <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  list       List all action servers');
  terminal.writeln('  type       Show the type of an action');
  terminal.writeln('  info       Show information about an action');
  terminal.writeln('  send_goal  Send a goal to an action server');
}

/**
 * Handle ros2 action list
 */
function handleList(args, terminal) {
  const showTypes = args.includes('-t') || args.includes('--show-types');
  const actions = SimDDS.getActions();

  if (actions.length === 0) {
    terminal.writeln('No action servers are currently available.');
    return;
  }

  for (const action of actions.sort((a, b) => a.name.localeCompare(b.name))) {
    if (showTypes) {
      terminal.writeln(`${action.name} [${action.type}]`);
    } else {
      terminal.writeln(action.name);
    }
  }
}

/**
 * Handle ros2 action type <action>
 */
function handleType(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 action type <action_name>');
    return;
  }

  const actionName = args[0];
  const actionInfo = SimDDS.getActionInfo(actionName);

  if (!actionInfo) {
    terminal.writeln(`\x1b[31mAction '${actionName}' not found\x1b[0m`);
    return;
  }

  terminal.writeln(actionInfo.type);
}

/**
 * Handle ros2 action info <action>
 */
function handleInfo(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 action info <action_name>');
    return;
  }

  const actionName = args[0];
  const actionInfo = SimDDS.getActionInfo(actionName);

  if (!actionInfo) {
    terminal.writeln(`\x1b[31mAction '${actionName}' not found\x1b[0m`);
    return;
  }

  terminal.writeln(`Action: ${actionName}`);
  terminal.writeln(`Type: ${actionInfo.type}`);
  terminal.writeln(`Node: ${actionInfo.node}`);
  terminal.writeln('');
  terminal.writeln('Action Servers:');
  terminal.writeln(`  ${actionInfo.node}`);
  terminal.writeln('');
  terminal.writeln('Action Clients:');
  terminal.writeln('  (none)');
}

/**
 * Handle ros2 action send_goal <action> <type> "<goal>" [--feedback]
 */
async function handleSendGoal(args, terminal) {
  // Parse arguments
  let actionName = null;
  let actionType = null;
  let goalData = null;
  let showFeedback = false;

  for (let i = 0; i < args.length; i++) {
    const arg = args[i];

    if (arg === '--feedback' || arg === '-f') {
      showFeedback = true;
    } else if (!actionName) {
      actionName = arg;
    } else if (!actionType) {
      actionType = arg;
    } else if (!goalData) {
      goalData = arg;
    }
  }

  if (!actionName || !actionType) {
    terminal.writeln('usage: ros2 action send_goal <action_name> <action_type> "<goal>" [--feedback]');
    return;
  }

  // Validate action type
  const actionDef = getAction(actionType);
  if (!actionDef) {
    terminal.writeln(`\x1b[31mUnknown action type: ${actionType}\x1b[0m`);
    terminal.writeln('');
    terminal.writeln('Available action types:');
    const matches = findActions(actionType.split('/').pop());
    for (const match of matches.slice(0, 10)) {
      terminal.writeln(`  ${match}`);
    }
    return;
  }

  // Check if action exists
  const actionInfo = SimDDS.getActionInfo(actionName);
  if (!actionInfo) {
    terminal.writeln(`\x1b[31mAction '${actionName}' not available\x1b[0m`);
    return;
  }

  // Parse goal data
  const data = goalData ? parseMessage(goalData) : {};
  const goal = actionDef.createGoal(data);

  terminal.writeln('Waiting for an action server to become available...');
  terminal.writeln('Sending goal:');
  terminal.writeln(formatMessage(goal));
  terminal.writeln('');

  try {
    let feedbackCallback = null;
    if (showFeedback) {
      feedbackCallback = (feedback) => {
        terminal.writeln('Feedback:');
        terminal.writeln(formatMessage(feedback));
        terminal.writeln('');
      };
    }

    terminal.writeln('Goal accepted with ID: goal_1');
    terminal.writeln('');

    const result = await SimDDS.sendActionGoal(actionName, actionType, goal, feedbackCallback);

    terminal.writeln('Result:');
    terminal.writeln(formatMessage(result));
    terminal.writeln('');
    terminal.writeln('Goal finished with status: SUCCEEDED');
  } catch (err) {
    terminal.writeln(`\x1b[31mAction failed: ${err.message}\x1b[0m`);
  }
}

// Self-register with the command registry
commandRegistry.registerRos2('action', handleRos2Action);

export default { handleRos2Action };
