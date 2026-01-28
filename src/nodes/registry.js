/**
 * Node Registry - Maps package/executable names to node classes
 * Nodes self-register by importing this singleton and calling register().
 */

class NodeRegistryClass {
  constructor() {
    this.packages = {};
  }

  /**
   * Get a node class by package and executable name
   * @param {string} packageName - Package name
   * @param {string} executableName - Executable name
   * @returns {Class|null} Node class or null if not found
   */
  getNode(packageName, executableName) {
    const pkg = this.packages[packageName];
    if (!pkg) return null;
    return pkg[executableName] || null;
  }

  /**
   * List all available packages
   * @returns {string[]} Package names
   */
  listPackages() {
    return Object.keys(this.packages);
  }

  /**
   * List executables in a package
   * @param {string} packageName - Package name
   * @returns {string[]} Executable names
   */
  listExecutables(packageName) {
    const pkg = this.packages[packageName];
    if (!pkg) return [];
    return Object.keys(pkg);
  }

  /**
   * Register a new node
   * @param {string} packageName - Package name
   * @param {string} executableName - Executable name
   * @param {Class} nodeClass - Node class
   */
  register(packageName, executableName, nodeClass) {
    if (!this.packages[packageName]) {
      this.packages[packageName] = {};
    }
    this.packages[packageName][executableName] = nodeClass;
  }

  /**
   * Check if a package exists
   * @param {string} packageName - Package name
   * @returns {boolean}
   */
  hasPackage(packageName) {
    return !!this.packages[packageName];
  }

  /**
   * Check if an executable exists
   * @param {string} packageName - Package name
   * @param {string} executableName - Executable name
   * @returns {boolean}
   */
  hasExecutable(packageName, executableName) {
    return !!this.getNode(packageName, executableName);
  }
}

export const nodeRegistry = new NodeRegistryClass();
