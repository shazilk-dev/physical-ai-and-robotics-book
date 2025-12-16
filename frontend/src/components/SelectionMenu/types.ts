/**
 * Type Definitions for SelectionMenu Component
 * Provides type safety for text selection and AI action features
 */

/**
 * Available AI actions for selected text
 */
export type ActionType = 'explain' | 'simplify' | 'example' | 'quiz';

/**
 * Context information for a text selection action
 */
export interface SelectionContext {
  /** The text that was selected by the user */
  text: string;
  /** The AI action to perform on the selected text */
  action: ActionType;
}

/**
 * Position coordinates for the selection menu
 */
export interface MenuPosition {
  /** Horizontal position in pixels */
  x: number;
  /** Vertical position in pixels */
  y: number;
}

/**
 * Configuration for action badges displayed in messages
 */
export interface ActionBadge {
  /** Emoji icon for the action */
  icon: string;
  /** Human-readable label */
  label: string;
  /** Color code for the badge background */
  color: string;
}

/**
 * Keyboard shortcut configuration
 */
export interface KeyboardShortcut {
  /** The key to press (combined with Cmd/Ctrl) */
  key: string;
  /** The action to trigger */
  action: ActionType;
  /** Display text for the shortcut */
  display: string;
}

/**
 * Selection menu configuration
 */
export interface SelectionMenuConfig {
  /** Minimum text length to show menu */
  minLength: number;
  /** Maximum text length to accept */
  maxLength: number;
  /** Delay in ms before detecting selection */
  selectionDelay: number;
  /** Whether keyboard shortcuts are enabled */
  keyboardShortcutsEnabled: boolean;
}

/**
 * Default configuration values
 */
export const DEFAULT_CONFIG: SelectionMenuConfig = {
  minLength: 10,
  maxLength: 500,
  selectionDelay: 10,
  keyboardShortcutsEnabled: true,
};
