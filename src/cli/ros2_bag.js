import { SimDDS } from '../core/SimDDS.js';
import { BagRecorder, BagStorage } from '../core/BagRecorder.js';
import { BagPlayer } from '../core/BagPlayer.js';
import { commandRegistry } from './commandRegistry.js';

/**
 * Handle ros2 bag commands
 */
export async function handleRos2Bag(args, terminal) {
  if (args.length === 0) {
    showHelp(terminal);
    terminal.finishCommand();
    return;
  }

  const subcommand = args[0];
  const subArgs = args.slice(1);

  switch (subcommand) {
    case 'record':
      handleRecord(subArgs, terminal);
      // Don't finish - runs until Ctrl+C
      break;

    case 'play':
      await handlePlay(subArgs, terminal);
      terminal.finishCommand();
      break;

    case 'info':
      handleInfo(subArgs, terminal);
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
  terminal.writeln('usage: ros2 bag <command> [options]');
  terminal.writeln('');
  terminal.writeln('Commands:');
  terminal.writeln('  record  Record topics to a bag');
  terminal.writeln('  play    Play back a recorded bag');
  terminal.writeln('  info    Show information about a bag');
}

/**
 * Handle ros2 bag record [-a] [-o name] <topics...>
 */
function handleRecord(args, terminal) {
  let recordAll = false;
  let bagName = `rosbag2_${Date.now()}`;
  const topics = [];

  for (let i = 0; i < args.length; i++) {
    const arg = args[i];

    if (arg === '-a' || arg === '--all') {
      recordAll = true;
    } else if (arg === '-o' || arg === '--output') {
      if (i + 1 < args.length) {
        bagName = args[++i];
      }
    } else if (!arg.startsWith('-')) {
      topics.push(arg);
    }
  }

  // Get topics to record
  let topicsToRecord = topics;

  if (recordAll) {
    topicsToRecord = SimDDS.getTopics().map(t => t.name);
    if (topicsToRecord.length === 0) {
      terminal.writeln('\x1b[33mNo topics available to record.\x1b[0m');
      terminal.finishCommand();
      return;
    }
  }

  if (topicsToRecord.length === 0) {
    terminal.writeln('usage: ros2 bag record [-a] [-o <bag_name>] <topic1> <topic2> ...');
    terminal.finishCommand();
    return;
  }

  // Create and start recorder
  const recorder = new BagRecorder(bagName, topicsToRecord);
  recorder.start();

  terminal.writeln(`\x1b[32m[INFO] Recording to '${bagName}'\x1b[0m`);
  terminal.writeln('Topics:');
  for (const topic of topicsToRecord) {
    terminal.writeln(`  ${topic}`);
  }
  terminal.writeln('');
  terminal.writeln('Press Ctrl+C to stop recording...');

  // Store recorder for Ctrl+C cleanup
  terminal.addBagRecorder({
    stop: () => {
      recorder.stop();
      BagStorage.store(recorder);

      const info = recorder.getInfo();
      terminal.writeln('');
      terminal.writeln(`\x1b[32m[INFO] Stopped recording.\x1b[0m`);
      terminal.writeln(`Bag: ${bagName}`);
      terminal.writeln(`Duration: ${info.duration?.toFixed(2) || 0}s`);
      terminal.writeln(`Messages: ${info.messageCount}`);
    }
  });
}

/**
 * Handle ros2 bag play <bag_name> [--rate N]
 */
async function handlePlay(args, terminal) {
  let bagName = null;
  let rate = 1.0;

  for (let i = 0; i < args.length; i++) {
    const arg = args[i];

    if (arg === '--rate' || arg === '-r') {
      if (i + 1 < args.length) {
        rate = parseFloat(args[++i]) || 1.0;
      }
    } else if (!arg.startsWith('-')) {
      bagName = arg;
    }
  }

  if (!bagName) {
    terminal.writeln('usage: ros2 bag play <bag_name> [--rate N]');
    terminal.writeln('');
    terminal.writeln('Available bags:');
    const bags = BagStorage.list();
    for (const name of bags) {
      terminal.writeln(`  ${name}`);
    }
    return;
  }

  // Load bag
  const recorder = BagStorage.get(bagName);
  if (!recorder) {
    terminal.writeln(`\x1b[31mBag '${bagName}' not found.\x1b[0m`);
    terminal.writeln('');
    terminal.writeln('Available bags:');
    const bags = BagStorage.list();
    for (const name of bags) {
      terminal.writeln(`  ${name}`);
    }
    return;
  }

  const info = recorder.getInfo();
  terminal.writeln(`\x1b[32m[INFO] Playing bag '${bagName}'\x1b[0m`);
  terminal.writeln(`Duration: ${info.duration?.toFixed(2) || 0}s`);
  terminal.writeln(`Rate: ${rate}x`);
  terminal.writeln('');

  // Play bag
  const player = new BagPlayer(recorder);

  return new Promise((resolve) => {
    player.onComplete = () => {
      terminal.writeln('');
      terminal.writeln('\x1b[32m[INFO] Playback complete.\x1b[0m');
      resolve();
    };

    player.onMessage = (msg) => {
      // Optionally show messages being played
      // terminal.writeln(`Playing: ${msg.topic}`);
    };

    player.play(rate);
  });
}

/**
 * Handle ros2 bag info <bag_name>
 */
function handleInfo(args, terminal) {
  if (args.length === 0) {
    terminal.writeln('usage: ros2 bag info <bag_name>');
    terminal.writeln('');
    terminal.writeln('Available bags:');
    const bags = BagStorage.list();
    for (const name of bags) {
      terminal.writeln(`  ${name}`);
    }
    return;
  }

  const bagName = args[0];
  const recorder = BagStorage.get(bagName);

  if (!recorder) {
    terminal.writeln(`\x1b[31mBag '${bagName}' not found.\x1b[0m`);
    return;
  }

  const info = recorder.getInfo();

  terminal.writeln(`Bag: ${info.name}`);
  terminal.writeln(`Duration: ${info.duration?.toFixed(2) || 0}s`);
  terminal.writeln(`Messages: ${info.messageCount}`);
  terminal.writeln('');
  terminal.writeln('Topics:');

  for (const topic of info.topics) {
    const count = info.topicCounts[topic] || 0;
    terminal.writeln(`  ${topic}: ${count} messages`);
  }
}

// Self-register with the command registry
commandRegistry.registerRos2('bag', handleRos2Bag);

export default { handleRos2Bag };
