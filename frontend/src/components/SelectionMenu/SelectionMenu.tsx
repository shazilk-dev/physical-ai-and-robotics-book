/**
 * SelectionMenu Component
 *
 * A professional, floating menu that appears when users select text from documentation.
 * Provides four AI-powered actions: Explain, Simplify, Example, and Quiz.
 *
 * Features:
 * - Smart selection detection (10-500 characters)
 * - Keyboard shortcuts (Cmd/Ctrl + E/S/X/Q)
 * - Mobile-responsive design
 * - Accessibility-compliant (ARIA labels, roles)
 * - Click-outside to dismiss
 * - Smooth animations
 *
 * @example
 * ```tsx
 * <SelectionMenu onAction={(context) => {
 *   console.log(`User wants to ${context.action}: ${context.text}`);
 * }} />
 * ```
 */

import React, { useState, useEffect, useCallback } from 'react';
import styles from './SelectionMenu.module.css';
import {
  SelectionContext,
  MenuPosition,
  DEFAULT_CONFIG,
} from './types';

/**
 * Props for the SelectionMenu component
 */
interface SelectionMenuProps {
  /** Callback function when an action is selected */
  onAction: (context: SelectionContext) => void;
  /** Optional configuration overrides */
  config?: Partial<typeof DEFAULT_CONFIG>;
}

// Re-export types for convenience
export type { SelectionContext, ActionType, MenuPosition } from './types';

/**
 * SelectionMenu Component Implementation
 */
export default function SelectionMenu({ onAction, config = {} }: SelectionMenuProps) {
  const menuConfig = { ...DEFAULT_CONFIG, ...config };
  const [selectedText, setSelectedText] = useState<string>('');
  const [position, setPosition] = useState<MenuPosition | null>(null);
  const [isVisible, setIsVisible] = useState<boolean>(false);

  const handleSelection = useCallback(() => {
    // Small delay to ensure selection is complete
    setTimeout(() => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // Only show menu if text meets length requirements
      if (text && text.length >= menuConfig.minLength && text.length <= menuConfig.maxLength) {
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          // Position menu above the selection, centered
          setSelectedText(text);
          setPosition({
            x: rect.left + rect.width / 2,
            y: rect.top - 10, // 10px above selection
          });
          setIsVisible(true);
        }
      } else {
        // Hide menu if selection is invalid
        setIsVisible(false);
        setPosition(null);
      }
    }, 10);
  }, []);

  const handleAction = (action: SelectionContext['action']) => {
    if (!selectedText) return;

    // Call parent handler with selected text and action
    onAction({
      text: selectedText,
      action,
    });

    // Clear selection and hide menu
    window.getSelection()?.removeAllRanges();
    setIsVisible(false);
    setPosition(null);
    setSelectedText('');
  };

  const handleClickOutside = useCallback((e: MouseEvent) => {
    const target = e.target as HTMLElement;

    // Hide menu if clicking outside of it
    if (!target.closest(`.${styles.selectionMenu}`)) {
      setIsVisible(false);
      setPosition(null);
    }
  }, []);

  // Handle keyboard shortcuts
  const handleKeyDown = useCallback((e: KeyboardEvent) => {
    if (!isVisible || !selectedText) return;

    const isMac = /Mac|iPod|iPhone|iPad/.test(navigator.platform);
    const modKey = isMac ? e.metaKey : e.ctrlKey;

    // Only trigger if modifier key (Cmd/Ctrl) is pressed
    if (!modKey) return;

    let action: SelectionContext['action'] | null = null;

    switch (e.key.toLowerCase()) {
      case 'e':
        action = 'explain';
        break;
      case 's':
        action = 'simplify';
        break;
      case 'x':
        action = 'example';
        break;
      case 'q':
        action = 'quiz';
        break;
      default:
        return; // Exit if key doesn't match
    }

    if (action) {
      e.preventDefault(); // Prevent default browser behavior
      handleAction(action);
    }
  }, [isVisible, selectedText]);

  useEffect(() => {
    // Listen for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection); // Mobile support

    // Listen for clicks outside
    document.addEventListener('mousedown', handleClickOutside);

    // Listen for keyboard shortcuts
    document.addEventListener('keydown', handleKeyDown);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleSelection, handleClickOutside, handleKeyDown]);

  if (!isVisible || !position) {
    return null;
  }

  return (
    <div
      className={styles.selectionMenu}
      style={{
        position: 'fixed',
        left: `${position.x}px`,
        top: `${position.y}px`,
        transform: 'translate(-50%, -100%)', // Center horizontally, position above
        zIndex: 999999, // Above everything
      }}
      role="menu"
      aria-label="Text selection actions"
    >
      <div className={styles.menuArrow} />
      <div className={styles.menuContent} role="group" aria-label="Action buttons">
        <button
          className={styles.menuButton}
          onClick={() => handleAction('explain')}
          title="Get a clear explanation (Cmd/Ctrl+E)"
          aria-label="Get a detailed explanation of the selected text"
          role="menuitem"
        >
          <span className={styles.icon} aria-hidden="true">🤖</span>
          <span className={styles.label}>Explain</span>
          <span className={styles.shortcut} aria-hidden="true">⌘E</span>
        </button>

        <button
          className={styles.menuButton}
          onClick={() => handleAction('simplify')}
          title="Simplify in easier terms (Cmd/Ctrl+S)"
          aria-label="Simplify the selected text in easier terms"
          role="menuitem"
        >
          <span className={styles.icon} aria-hidden="true">📝</span>
          <span className={styles.label}>Simplify</span>
          <span className={styles.shortcut} aria-hidden="true">⌘S</span>
        </button>

        <button
          className={styles.menuButton}
          onClick={() => handleAction('example')}
          title="Show practical example (Cmd/Ctrl+X)"
          aria-label="Show a practical code example of the selected concept"
          role="menuitem"
        >
          <span className={styles.icon} aria-hidden="true">💡</span>
          <span className={styles.label}>Example</span>
          <span className={styles.shortcut} aria-hidden="true">⌘X</span>
        </button>

        <button
          className={styles.menuButton}
          onClick={() => handleAction('quiz')}
          title="Quiz me on this (Cmd/Ctrl+Q)"
          aria-label="Generate quiz questions about the selected concept"
          role="menuitem"
        >
          <span className={styles.icon} aria-hidden="true">❓</span>
          <span className={styles.label}>Quiz Me</span>
          <span className={styles.shortcut} aria-hidden="true">⌘Q</span>
        </button>
      </div>

      <div className={styles.hint}>
        <span className={styles.selectedTextPreview}>
          {selectedText.length > 50
            ? selectedText.substring(0, 47) + '...'
            : selectedText}
        </span>
      </div>
    </div>
  );
}
