import { SimDDS } from '../core/SimDDS.js';
import { parseMessage, formatMessage } from './messageParser.js';
import { getMessage, findMessages } from '../msgs/index.js';
import { commandRegistry } from './commandRegistry.js';
import { formatQoS, parseQoSArgs } from '../utils/qos.js';

/**
 * Handle ros2 topic commands
 */
export async function handleRos2Topic(args, terminal) {
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

    case 'info':
      handleInfo(subArgs, terminal);
      terminal.finishCommand();
      break;

    case 'type':
      handleType(subArgs, terminal);
      terminal.finishCommand();
      break;

    case 'find':
      handleFind(subArgs, terminal);
      terminal.finishCommand();
      break;

    case 'echo':
      handleEcho(subArgs, terminal);
      // Don't finish command - runs until Ctrl+C
      break;

    case 'pub':
      await handlePub(subArgs, terminal);
      break;

    case 'hz':
      handleHz(subArgs, terminal);
      // Don't finish command - runs until Ctrl+C
      break;

    case 'bw':
      handleBw(subArgs, terminal);
      // Don't finish command - runs until Ctrl+C
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
  terminal.writeln('usage: ros2 topic <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  list    List all active topics');
  terminal.writeln('  info    Show information about a topic');
  terminal.writeln('  type    Show the type of a topic');
  terminal.writeln('  find    Find topics by type');
  terminal.writeln('  echo    Subscribe and print messages');
  terminal.writeln('  pub     Publish a message to a topic');
  terminal.writeln('  hz      Show message rate for a topic');
  terminal.writeln('  bw      Show bandwidth for a topic');
}

/**
 * Handle ros2 topic list
 */
function handleList(args, terminal) {
  const showTypes = args.includes('-t') || args.includes('--show-types');
  const topics = SimDDS.getTopics();

  if (topics.length === 0) {
    terminal.writeln('No topics are currently active.');
    return;
  }

  for (const topic of topics.sort((a, b) => a.name.localeCompare(b.name))) {
    if (showTypes) {
      terminal.writeln(`${topic.name} [${topic.type}]`);
    } else {
      terminal.writeln(topic.name);
    }
  }
}

/**
 * Handle ros2 topic info <topic>
 */
function handleInfo(args, terminal) {
  const verbose = args.includes('--verbose') || args.includes('-v');
  const topicName = args.find(a => !a.startsWith('-'));

  if (!topicName) {
    terminal.writeln('usage: ros2 topic info <topic_name> [--verbose]');
    return;
  }

  const topicInfo = SimDDS.getTopicInfo(topicName);
  if (!topicInfo) {
    terminal.writeln(`\x1b[31mTopic '${topicName}' not found\x1b[0m`);
    return;
  }

  terminal.writeln(`Type: ${topicInfo.type}`);
  terminal.writeln(`Publisher count: ${topicInfo.publishers.length}`);
  terminal.writeln(`Subscription count: ${topicInfo.subscribers.length}`);

  if (verbose) {
    terminal.writeln('');
    terminal.writeln('Publishers:');
    for (const pub of topicInfo.publishers) {
      terminal.writeln(`  Node: ${pub.nodeId}`);
      for (const line of formatQoS(pub.qos, '    ')) {
        terminal.writeln(line);
      }
    }
    terminal.writeln('');
    terminal.writeln('Subscribers:');
    for (const sub of topicInfo.subscribers) {
      terminal.writeln(`  Node: ${sub.nodeId || '(anonymous)'}`);
      for (const line of formatQoS(sub.qos, '    ')) {
        terminal.writeln(line);
      }
    }
  }
}

/**
 * Handle ros2 topic type <topic>
 */
function handleType(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 topic type <topic_name>');
    return;
  }

  const topicName = args[0];
  const topicInfo = SimDDS.getTopicInfo(topicName);

  if (!topicInfo) {
    terminal.writeln(`\x1b[31mTopic '${topicName}' not found\x1b[0m`);
    return;
  }

  terminal.writeln(topicInfo.type);
}

/**
 * Handle ros2 topic find <type>
 */
function handleFind(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 topic find <msg_type>');
    return;
  }

  const msgType = args[0];
  const topics = SimDDS.getTopics();
  const matching = topics.filter(t => t.type === msgType);

  if (matching.length === 0) {
    terminal.writeln(`No topics of type '${msgType}' found.`);
    return;
  }

  for (const topic of matching) {
    terminal.writeln(topic.name);
  }
}

/**
 * Handle ros2 topic echo <topic>
 */
function handleEcho(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 topic echo <topic_name> [--qos-*]');
    terminal.finishCommand();
    return;
  }

  // Parse QoS flags
  const { qos, remaining, error } = parseQoSArgs(args);
  if (error) {
    terminal.writeln(`\x1b[31m${error}\x1b[0m`);
    terminal.finishCommand();
    return;
  }

  const topicName = remaining[0];
  if (!topicName) {
    terminal.writeln('usage: ros2 topic echo <topic_name> [--qos-*]');
    terminal.finishCommand();
    return;
  }

  let topicInfo = SimDDS.getTopicInfo(topicName);
  let sub = null;
  let pollInterval = null;

  // Helper to subscribe with correct type
  const subscribeToTopic = (type) => {
    // Unsubscribe from previous subscription if any
    if (sub) {
      SimDDS.unsubscribe(sub);
    }
    sub = SimDDS.subscribe(topicName, type, (msg) => {
      terminal.writeln('---');
      terminal.writeln(formatMessage(msg));
    }, qos);
  };

  if (!topicInfo) {
    // Topic doesn't exist yet - show clear message and poll for availability
    terminal.writeln(`\x1b[33mTopic '${topicName}' does not exist yet.\x1b[0m`);
    terminal.writeln(`\x1b[33mWill start echoing when a publisher creates it...\x1b[0m`);

    // Poll every second to check if topic becomes available
    pollInterval = setInterval(() => {
      const newTopicInfo = SimDDS.getTopicInfo(topicName);
      if (newTopicInfo && newTopicInfo.type !== 'unknown') {
        // Topic is now available - resubscribe with correct type
        terminal.writeln(`\x1b[32mTopic '${topicName}' is now available [${newTopicInfo.type}]\x1b[0m`);
        clearInterval(pollInterval);
        pollInterval = null;
        subscribeToTopic(newTopicInfo.type);
      }
    }, 1000);

    // Subscribe with unknown type initially (won't receive messages)
    subscribeToTopic('unknown');
  } else {
    // Topic exists - subscribe normally
    subscribeToTopic(topicInfo.type);
  }

  // Store subscription for Ctrl+C cleanup
  terminal.addSubscription({
    destroy: () => {
      if (pollInterval) {
        clearInterval(pollInterval);
      }
      if (sub) {
        SimDDS.unsubscribe(sub);
      }
    }
  });
}

/**
 * Handle ros2 topic pub <topic> <type> "<data>"
 */
async function handlePub(args, terminal) {
  // Parse QoS flags first
  const { qos, remaining, error } = parseQoSArgs(args);
  if (error) {
    terminal.writeln(`\x1b[31m${error}\x1b[0m`);
    terminal.finishCommand();
    return;
  }

  // Parse remaining arguments
  let once = false;
  let rate = 1;
  let waitCount = 0;
  let topicName = null;
  let msgType = null;
  let msgData = null;

  for (let i = 0; i < remaining.length; i++) {
    const arg = remaining[i];

    if (arg === '--once' || arg === '-1') {
      once = true;
    } else if (arg === '-r' || arg === '--rate') {
      rate = parseFloat(remaining[++i]) || 1;
    } else if (arg === '-w' || arg === '--wait') {
      waitCount = parseInt(remaining[++i]) || 0;
    } else if (!topicName) {
      topicName = arg;
    } else if (!msgType) {
      msgType = arg;
    } else if (!msgData) {
      msgData = arg;
    }
  }

  if (!topicName || !msgType) {
    terminal.writeln('usage: ros2 topic pub [--once] [-r RATE] <topic> <msg_type> "<data>" [--qos-*]');
    terminal.finishCommand();
    return;
  }

  // Validate message type
  const msgDef = getMessage(msgType);
  if (!msgDef) {
    terminal.writeln(`\x1b[31mUnknown message type: ${msgType}\x1b[0m`);
    terminal.writeln('');
    terminal.writeln('Available message types matching this pattern:');
    const matches = findMessages(msgType.split('/').pop());
    for (const match of matches.slice(0, 10)) {
      terminal.writeln(`  ${match}`);
    }
    terminal.finishCommand();
    return;
  }

  // Parse message data
  const data = msgData ? parseMessage(msgData) : {};
  const message = msgDef.create(data);

  // Register a tracked publisher so QoS shows in topic info --verbose
  const pubId = SimDDS.comm.advertise(topicName, msgType, '/_cli_pub', qos);

  terminal.writeln(`publisher: beginning loop`);
  terminal.writeln(`publishing #1: ${JSON.stringify(message)}`);

  // Publish first message
  SimDDS.publish(topicName, msgType, message);

  if (once) {
    SimDDS.comm.unadvertise(pubId);
    terminal.finishCommand();
    return;
  }

  // Set up repeating publish
  let count = 1;
  const interval = setInterval(() => {
    count++;
    terminal.writeln(`publishing #${count}: ${JSON.stringify(message)}`);
    SimDDS.publish(topicName, msgType, message);
  }, 1000 / rate);

  // Store for Ctrl+C cleanup
  terminal.addSubscription({
    destroy: () => {
      clearInterval(interval);
      SimDDS.comm.unadvertise(pubId);
    }
  });
}

/**
 * Handle ros2 topic hz <topic>
 */
function handleHz(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 topic hz <topic_name>');
    terminal.finishCommand();
    return;
  }

  const topicName = args[0];
  const topicInfo = SimDDS.getTopicInfo(topicName);

  if (!topicInfo) {
    terminal.writeln(`\x1b[33mWaiting for topic '${topicName}'...\x1b[0m`);
  }

  const timestamps = [];
  const windowSize = 100;

  const sub = SimDDS.subscribe(topicName, topicInfo?.type || 'unknown', () => {
    const now = Date.now();
    timestamps.push(now);

    // Keep only recent timestamps
    while (timestamps.length > windowSize) {
      timestamps.shift();
    }

    if (timestamps.length >= 2) {
      const duration = Math.max((timestamps[timestamps.length - 1] - timestamps[0]) / 1000, 0.001);
      const rate = (timestamps.length - 1) / duration;
      const avgPeriod = duration / (timestamps.length - 1);

      terminal.writeln(`average rate: ${rate.toFixed(3)}`);
      terminal.writeln(`  min: ${(avgPeriod * 0.9).toFixed(3)}s max: ${(avgPeriod * 1.1).toFixed(3)}s std dev: ${(avgPeriod * 0.05).toFixed(5)}s window: ${timestamps.length}`);
    }
  });

  terminal.addSubscription({
    destroy: () => SimDDS.unsubscribe(sub)
  });
}

/**
 * Format byte count for display (matches ROS 2 CLI format)
 */
function formatBytes(bytes) {
  if (bytes >= 1000000) {
    return `${(bytes / 1000000).toFixed(2)} MB`;
  } else if (bytes >= 1000) {
    return `${(bytes / 1000).toFixed(2)} KB`;
  }
  return `${Math.round(bytes)} B`;
}

/**
 * Handle ros2 topic bw <topic>
 */
function handleBw(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 topic bw <topic_name>');
    terminal.finishCommand();
    return;
  }

  const topicName = args[0];
  const topicInfo = SimDDS.getTopicInfo(topicName);

  if (!topicInfo) {
    terminal.writeln(`\x1b[33mWaiting for topic '${topicName}'...\x1b[0m`);
  }

  const samples = [];
  const windowSize = 100;
  let subscribedPrinted = false;

  const sub = SimDDS.subscribe(topicName, topicInfo?.type || 'unknown', (msg) => {
    const now = Date.now();
    const size = JSON.stringify(msg).length;
    samples.push({ time: now, size });

    while (samples.length > windowSize) {
      samples.shift();
    }

    if (!subscribedPrinted) {
      terminal.writeln(`Subscribed to [${topicName}]`);
      subscribedPrinted = true;
    }

    if (samples.length >= 2) {
      const duration = Math.max((samples[samples.length - 1].time - samples[0].time) / 1000, 0.001);
      const totalBytes = samples.reduce((sum, s) => sum + s.size, 0);
      const bw = totalBytes / duration;
      const meanSize = totalBytes / samples.length;
      const minSize = Math.min(...samples.map(s => s.size));
      const maxSize = Math.max(...samples.map(s => s.size));

      terminal.writeln(`${formatBytes(bw)}/s from ${samples.length} messages`);
      terminal.writeln(`  Message size mean: ${formatBytes(meanSize)} min: ${formatBytes(minSize)} max: ${formatBytes(maxSize)}`);
    }
  });

  terminal.addSubscription({
    destroy: () => SimDDS.unsubscribe(sub)
  });
}

// Self-register with the command registry
commandRegistry.registerRos2('topic', handleRos2Topic);

export default { handleRos2Topic };
