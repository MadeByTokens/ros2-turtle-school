import { Terminal as XTerm } from 'xterm';
import { FitAddon } from '@xterm/addon-fit';

/**
 * Terminal - Single xterm.js instance wrapper with command history and input handling
 */
export class Terminal {
  constructor(container, options = {}) {
    this.container = container;
    this.id = options.id || `term_${Date.now()}`;
    this.onCommand = options.onCommand || (() => {});
    this.onInterrupt = options.onInterrupt || (() => {});

    // Command history
    this.history = [];
    this.historyIndex = -1;
    this.maxHistory = 100;

    // Current input state
    this.currentLine = '';
    this.cursorPosition = 0;
    this.prompt = 'ros2@sim:~$ ';

    // State flags
    this.isProcessing = false;
    this.waitingForInput = false;
    this.focused = false;

    // Teleop state tracking
    this.teleopActive = false;
    this.teleopType = null;

    // Active continuous commands (echo, pub, etc.)
    this.activeSubscriptions = [];
    this.activeBagRecorders = [];

    // Initialize xterm
    this._initTerminal();
  }

  _initTerminal() {
    this.xterm = new XTerm({
      theme: {
        background: '#1e1e1e',
        foreground: '#d4d4d4',
        cursor: '#ffffff',
        cursorAccent: '#1e1e1e',
        selection: 'rgba(255, 255, 255, 0.3)',
        black: '#000000',
        red: '#cd3131',
        green: '#0dbc79',
        yellow: '#e5e510',
        blue: '#2472c8',
        magenta: '#bc3fbc',
        cyan: '#11a8cd',
        white: '#e5e5e5',
        brightBlack: '#666666',
        brightRed: '#f14c4c',
        brightGreen: '#23d18b',
        brightYellow: '#f5f543',
        brightBlue: '#3b8eea',
        brightMagenta: '#d670d6',
        brightCyan: '#29b8db',
        brightWhite: '#ffffff'
      },
      fontFamily: 'Menlo, Monaco, "Courier New", monospace',
      fontSize: 14,
      lineHeight: 1.2,
      cursorBlink: true,
      cursorStyle: 'block',
      scrollback: 1000,
      allowTransparency: true
    });

    this.fitAddon = new FitAddon();
    this.xterm.loadAddon(this.fitAddon);

    this.xterm.open(this.container);
    this.fit();

    // Handle input
    this.xterm.onData(this._handleInput.bind(this));

    // Handle focus using DOM events on the terminal element
    this.container.addEventListener('focus', () => {
      this.focused = true;
    }, true);

    this.container.addEventListener('blur', () => {
      this.focused = false;
    }, true);

    // Show initial prompt
    this.writePrompt();

    // Listen for teleop events
    this._setupTeleopListeners();
  }

  _setupTeleopListeners() {
    // Listen for teleop activation
    window.addEventListener('teleop-active', (event) => {
      this.teleopActive = true;
      this.teleopType = event.detail?.type || 'wasd-keys';
    });

    // Listen for teleop deactivation
    window.addEventListener('teleop-inactive', () => {
      this.teleopActive = false;
      this.teleopType = null;
    });
  }

  /**
   * Forward a key to window as a synthetic KeyboardEvent for teleop
   */
  _forwardKeyToWindow(key, code) {
    // Dispatch keydown event
    const keydownEvent = new KeyboardEvent('keydown', {
      key,
      code,
      bubbles: true,
      cancelable: true
    });
    window.dispatchEvent(keydownEvent);

    // Schedule keyup after a short delay
    setTimeout(() => {
      const keyupEvent = new KeyboardEvent('keyup', {
        key,
        code,
        bubbles: true,
        cancelable: true
      });
      window.dispatchEvent(keyupEvent);
    }, 50);
  }

  _handleInput(data) {
    // Handle special keys
    for (let i = 0; i < data.length; i++) {
      const char = data.charCodeAt(i);

      // Ctrl+C (interrupt)
      if (char === 3) {
        this._handleInterrupt();
        continue;
      }

      // Ctrl+D (EOF - ignore for now)
      if (char === 4) {
        continue;
      }

      // Ctrl+L (clear screen)
      if (char === 12) {
        this.clear();
        this.writePrompt();
        this._refreshLine();
        continue;
      }

      // Enter
      if (char === 13) {
        this._handleEnter();
        continue;
      }

      // Backspace
      if (char === 127 || char === 8) {
        this._handleBackspace();
        continue;
      }

      // Tab (completion)
      if (char === 9) {
        this._handleTab();
        continue;
      }

      // Escape sequences (arrows, etc.)
      if (char === 27) {
        if (i + 2 < data.length && data.charCodeAt(i + 1) === 91) {
          const code = data.charCodeAt(i + 2);

          // Up arrow
          if (code === 65) {
            // Forward arrow keys to teleop if active with arrow-keys type
            if (this.teleopActive && this.teleopType === 'arrow-keys') {
              this._forwardKeyToWindow('ArrowUp', 'ArrowUp');
              i += 2;
              continue;
            }
            this._historyUp();
            i += 2;
            continue;
          }

          // Down arrow
          if (code === 66) {
            if (this.teleopActive && this.teleopType === 'arrow-keys') {
              this._forwardKeyToWindow('ArrowDown', 'ArrowDown');
              i += 2;
              continue;
            }
            this._historyDown();
            i += 2;
            continue;
          }

          // Right arrow
          if (code === 67) {
            if (this.teleopActive && this.teleopType === 'arrow-keys') {
              this._forwardKeyToWindow('ArrowRight', 'ArrowRight');
              i += 2;
              continue;
            }
            this._cursorRight();
            i += 2;
            continue;
          }

          // Left arrow
          if (code === 68) {
            if (this.teleopActive && this.teleopType === 'arrow-keys') {
              this._forwardKeyToWindow('ArrowLeft', 'ArrowLeft');
              i += 2;
              continue;
            }
            this._cursorLeft();
            i += 2;
            continue;
          }

          // Home (Ctrl+A behavior with ^[[H)
          if (code === 72) {
            this._cursorHome();
            i += 2;
            continue;
          }

          // End (Ctrl+E behavior with ^[[F)
          if (code === 70) {
            this._cursorEnd();
            i += 2;
            continue;
          }

          // Delete key ^[[3~
          if (code === 51 && i + 3 < data.length && data.charCodeAt(i + 3) === 126) {
            this._handleDelete();
            i += 3;
            continue;
          }
        }
        continue;
      }

      // Regular character
      if (char >= 32 && char < 127) {
        // Check for teleop WASD keys when teleop is active
        if (this.teleopActive && this.teleopType === 'wasd-keys') {
          const upperChar = data[i].toUpperCase();
          if ('WASDQE'.includes(upperChar)) {
            // Forward WASD/QE keys to window for teleop control
            this._forwardKeyToWindow(upperChar, `Key${upperChar}`);
            continue; // Skip normal terminal processing
          }
        }
        this._insertChar(data[i]);
      }
    }
  }

  _insertChar(char) {
    if (this.isProcessing) return;

    // Insert character at cursor position
    this.currentLine =
      this.currentLine.slice(0, this.cursorPosition) +
      char +
      this.currentLine.slice(this.cursorPosition);
    this.cursorPosition++;

    this._refreshLine();
  }

  _handleBackspace() {
    if (this.isProcessing) return;
    if (this.cursorPosition === 0) return;

    this.currentLine =
      this.currentLine.slice(0, this.cursorPosition - 1) +
      this.currentLine.slice(this.cursorPosition);
    this.cursorPosition--;

    this._refreshLine();
  }

  _handleDelete() {
    if (this.isProcessing) return;
    if (this.cursorPosition >= this.currentLine.length) return;

    this.currentLine =
      this.currentLine.slice(0, this.cursorPosition) +
      this.currentLine.slice(this.cursorPosition + 1);

    this._refreshLine();
  }

  _handleEnter() {
    if (this.isProcessing) return;

    this.xterm.write('\r\n');

    const command = this.currentLine.trim();
    this.currentLine = '';
    this.cursorPosition = 0;

    if (command) {
      // Add to history
      if (this.history[this.history.length - 1] !== command) {
        this.history.push(command);
        if (this.history.length > this.maxHistory) {
          this.history.shift();
        }
      }
      this.historyIndex = this.history.length;

      // Execute command
      this.isProcessing = true;
      this.onCommand(command, this);
    } else {
      this.writePrompt();
    }
  }

  _handleInterrupt() {
    // Cancel any active subscriptions
    for (const sub of this.activeSubscriptions) {
      if (sub.destroy) {
        sub.destroy();
      }
    }
    this.activeSubscriptions = [];

    // Stop any active bag recorders
    for (const recorder of this.activeBagRecorders) {
      if (recorder.stop) {
        recorder.stop();
      }
    }
    this.activeBagRecorders = [];

    // Call interrupt handler
    this.onInterrupt(this);

    // Show ^C and new prompt
    this.xterm.write('^C\r\n');
    this.currentLine = '';
    this.cursorPosition = 0;
    this.isProcessing = false;
    this.writePrompt();
  }

  _handleTab() {
    // Tab completion - delegate to command handler
    // For now, just insert spaces
    this._insertChar(' ');
    this._insertChar(' ');
  }

  _historyUp() {
    if (this.isProcessing) return;
    if (this.historyIndex <= 0) return;

    this.historyIndex--;
    this.currentLine = this.history[this.historyIndex];
    this.cursorPosition = this.currentLine.length;
    this._refreshLine();
  }

  _historyDown() {
    if (this.isProcessing) return;

    if (this.historyIndex >= this.history.length - 1) {
      this.historyIndex = this.history.length;
      this.currentLine = '';
      this.cursorPosition = 0;
      this._refreshLine();
      return;
    }

    this.historyIndex++;
    this.currentLine = this.history[this.historyIndex];
    this.cursorPosition = this.currentLine.length;
    this._refreshLine();
  }

  _cursorLeft() {
    if (this.cursorPosition > 0) {
      this.cursorPosition--;
      this.xterm.write('\x1b[D');
    }
  }

  _cursorRight() {
    if (this.cursorPosition < this.currentLine.length) {
      this.cursorPosition++;
      this.xterm.write('\x1b[C');
    }
  }

  _cursorHome() {
    while (this.cursorPosition > 0) {
      this.cursorPosition--;
      this.xterm.write('\x1b[D');
    }
  }

  _cursorEnd() {
    while (this.cursorPosition < this.currentLine.length) {
      this.cursorPosition++;
      this.xterm.write('\x1b[C');
    }
  }

  _refreshLine() {
    // Move to start of input, clear line, rewrite
    const promptLen = this.prompt.length;

    // Move cursor to start of prompt
    this.xterm.write('\r');
    // Clear line
    this.xterm.write('\x1b[K');
    // Write prompt and current line
    this.xterm.write(this.prompt + this.currentLine);

    // Move cursor to correct position
    const backspaces = this.currentLine.length - this.cursorPosition;
    if (backspaces > 0) {
      this.xterm.write(`\x1b[${backspaces}D`);
    }
  }

  /**
   * Write text to terminal
   */
  write(text) {
    this.xterm.write(text);
  }

  /**
   * Write a line (with newline)
   */
  writeln(text) {
    this.xterm.write(text + '\r\n');
  }

  /**
   * Write the prompt
   */
  writePrompt() {
    this.xterm.write(this.prompt);
  }

  /**
   * Set custom prompt
   */
  setPrompt(prompt) {
    this.prompt = prompt;
  }

  /**
   * Finish processing and show prompt
   */
  finishCommand() {
    this.isProcessing = false;
    this.writePrompt();
  }

  /**
   * Clear the terminal
   */
  clear() {
    this.xterm.clear();
  }

  /**
   * Fit terminal to container
   */
  fit() {
    try {
      this.fitAddon.fit();
    } catch (e) {
      // Ignore fit errors
    }
  }

  /**
   * Focus the terminal
   */
  focus() {
    this.xterm.focus();
  }

  /**
   * Add an active subscription (for Ctrl+C cleanup)
   */
  addSubscription(sub) {
    this.activeSubscriptions.push(sub);
  }

  /**
   * Remove a subscription
   */
  removeSubscription(sub) {
    const idx = this.activeSubscriptions.indexOf(sub);
    if (idx >= 0) {
      this.activeSubscriptions.splice(idx, 1);
    }
  }

  /**
   * Add an active bag recorder (for Ctrl+C cleanup)
   */
  addBagRecorder(recorder) {
    this.activeBagRecorders.push(recorder);
  }

  /**
   * Remove a bag recorder
   */
  removeBagRecorder(recorder) {
    const idx = this.activeBagRecorders.indexOf(recorder);
    if (idx >= 0) {
      this.activeBagRecorders.splice(idx, 1);
    }
  }

  /**
   * Destroy the terminal
   */
  destroy() {
    // Cleanup subscriptions
    for (const sub of this.activeSubscriptions) {
      if (sub.destroy) {
        sub.destroy();
      }
    }
    this.activeSubscriptions = [];

    // Cleanup bag recorders
    for (const recorder of this.activeBagRecorders) {
      if (recorder.stop) {
        recorder.stop();
      }
    }
    this.activeBagRecorders = [];

    // Dispose xterm
    this.xterm.dispose();
  }
}
