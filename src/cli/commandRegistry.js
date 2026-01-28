/**
 * Command Registry - Central registry for CLI commands.
 * Command handlers self-register by importing this singleton and calling register().
 */

class CommandRegistryClass {
  constructor() {
    this.commands = new Map();
    this.ros2Commands = new Map();
  }

  /**
   * Register a top-level command
   * @param {string} name - Command name (e.g., 'clear', 'ls', 'rqt_graph')
   * @param {Function} handler - Handler function (args, terminal) => void
   */
  register(name, handler) {
    this.commands.set(name, handler);
  }

  /**
   * Register a ros2 subcommand
   * @param {string} name - Subcommand name (e.g., 'run', 'topic', 'node')
   * @param {Function} handler - Handler function (args, terminal) => void
   */
  registerRos2(name, handler) {
    this.ros2Commands.set(name, handler);
  }

  /**
   * Check if a command exists
   */
  hasCommand(name) {
    return this.commands.has(name);
  }

  /**
   * Check if a ros2 subcommand exists
   */
  hasRos2Command(name) {
    return this.ros2Commands.has(name);
  }

  /**
   * Execute a top-level command
   * @returns {boolean} True if command was handled
   */
  async execute(name, args, terminal) {
    const handler = this.commands.get(name);
    if (!handler) return false;
    await handler(args, terminal);
    return true;
  }

  /**
   * Execute a ros2 subcommand
   * @returns {boolean} True if command was handled
   */
  async executeRos2(name, args, terminal) {
    const handler = this.ros2Commands.get(name);
    if (!handler) return false;
    await handler(args, terminal);
    return true;
  }

  /**
   * List all registered top-level commands
   */
  listCommands() {
    return Array.from(this.commands.keys());
  }

  /**
   * List all registered ros2 subcommands
   */
  listRos2Commands() {
    return Array.from(this.ros2Commands.keys());
  }
}

export const commandRegistry = new CommandRegistryClass();
