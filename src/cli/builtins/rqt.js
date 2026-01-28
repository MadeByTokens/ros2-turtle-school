/**
 * RQT commands - rqt, rqt_graph, rqt_console
 */

import { commandRegistry } from '../commandRegistry.js';
import { Events } from '../../core/Events.js';

/**
 * Handle rqt command - open tool selector
 */
function handleRqt(args, terminal) {
  window.dispatchEvent(new CustomEvent(Events.OPEN_RQT_MODAL));
  terminal.finishCommand();
  return true;
}

/**
 * Handle rqt_graph command
 */
function handleRqtGraph(args, terminal) {
  window.dispatchEvent(new CustomEvent(Events.TOGGLE_GRAPH));
  terminal.writeln('Opening rqt_graph...');
  terminal.finishCommand();
  return true;
}

/**
 * Handle rqt_console command
 */
function handleRqtConsole(args, terminal) {
  window.dispatchEvent(new CustomEvent(Events.TOGGLE_CONSOLE));
  terminal.writeln('Opening rqt_console...');
  terminal.finishCommand();
  return true;
}

// Register rqt commands
commandRegistry.register('rqt', handleRqt);
commandRegistry.register('rqt_graph', handleRqtGraph);
commandRegistry.register('rqt_console', handleRqtConsole);
