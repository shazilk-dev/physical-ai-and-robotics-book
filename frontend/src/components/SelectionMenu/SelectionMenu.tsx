/**
 * SelectionMenu Component
 * Shows a floating menu when user selects text with AI action options
 */

import React, { useState, useEffect, useCallback } from 'react';
import styles from './SelectionMenu.module.css';

export interface SelectionContext {
  text: string;
  action: 'explain' | 'simplify' | 'example' | 'quiz';
}

interface SelectionMenuProps {
  onAction: (context: SelectionContext) => void;
}

export default function SelectionMenu({ onAction }: SelectionMenuProps) {
  const [selectedText, setSelectedText] = useState<string>('');
  const [position, setPosition] = useState<{ x: number; y: number } | null>(null);
  const [isVisible, setIsVisible] = useState(false);

  const handleSelection = useCallback(() => {
    // Small delay to ensure selection is complete
    setTimeout(() => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // Only show menu if:
      // 1. Text is selected
      // 2. Text is at least 10 characters (avoid accidental selections)
      // 3. Text is not too long (max 500 chars)
      if (text && text.length >= 10 && text.length <= 500) {
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

  useEffect(() => {
    // Listen for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection); // Mobile support

    // Listen for clicks outside
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [handleSelection, handleClickOutside]);

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
    >
      <div className={styles.menuArrow} />
      <div className={styles.menuContent}>
        <button
          className={styles.menuButton}
          onClick={() => handleAction('explain')}
          title="Get a clear explanation of this concept"
        >
          <span className={styles.icon}>🤖</span>
          <span className={styles.label}>Explain</span>
        </button>

        <button
          className={styles.menuButton}
          onClick={() => handleAction('simplify')}
          title="Simplify this in easier terms"
        >
          <span className={styles.icon}>📝</span>
          <span className={styles.label}>Simplify</span>
        </button>

        <button
          className={styles.menuButton}
          onClick={() => handleAction('example')}
          title="Show me a practical example"
        >
          <span className={styles.icon}>💡</span>
          <span className={styles.label}>Example</span>
        </button>

        <button
          className={styles.menuButton}
          onClick={() => handleAction('quiz')}
          title="Quiz me on this concept"
        >
          <span className={styles.icon}>❓</span>
          <span className={styles.label}>Quiz Me</span>
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
