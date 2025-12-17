/**
 * ChatSettingsPanel Component
 *
 * Provides UI for users to customize their AI learning experience.
 * Includes response mode selection, explanation depth slider, and preferences.
 *
 * @example
 * ```tsx
 * <ChatSettingsPanel
 *   settings={settings}
 *   onUpdate={setSettings}
 *   onClose={() => setShowSettings(false)}
 * />
 * ```
 */

import React from 'react';
import { Settings, X } from 'lucide-react';
import styles from './ChatSettingsPanel.module.css';
import {
  ChatSettings,
  RESPONSE_MODES,
  DEPTH_LABELS,
  LANGUAGE_STYLES,
  ResponseMode,
  LanguageStyle,
} from './types';

interface ChatSettingsPanelProps {
  /** Current settings */
  settings: ChatSettings;
  /** Callback when settings change */
  onUpdate: (settings: ChatSettings) => void;
  /** Callback to close the panel */
  onClose: () => void;
}

export default function ChatSettingsPanel({
  settings,
  onUpdate,
  onClose,
}: ChatSettingsPanelProps) {
  const handleModeChange = (mode: ResponseMode) => {
    onUpdate({ ...settings, responseMode: mode });
  };

  const handleDepthChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    onUpdate({ ...settings, explanationDepth: parseInt(e.target.value) });
  };

  const handleCodeExamplesChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    onUpdate({ ...settings, includeCodeExamples: e.target.checked });
  };

  const handleVisualsChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    onUpdate({ ...settings, includeVisuals: e.target.checked });
  };

  const handleLanguageStyleChange = (style: LanguageStyle) => {
    onUpdate({ ...settings, languageStyle: style });
  };

  const handleReset = () => {
    const defaultSettings: ChatSettings = {
      responseMode: 'detailed',
      explanationDepth: 3,
      includeCodeExamples: true,
      includeVisuals: true,
      languageStyle: 'casual',
    };
    onUpdate(defaultSettings);
  };

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.panel} onClick={(e) => e.stopPropagation()}>
        {/* Header */}
        <div className={styles.header}>
          <h2 className={styles.title}>
            <Settings className={styles.icon} size={20} aria-hidden="true" />
            Chat Settings
          </h2>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close settings"
          >
            <X size={20} />
          </button>
        </div>

        {/* Content */}
        <div className={styles.content}>
          {/* Response Mode */}
          <div className={styles.section}>
            <label className={styles.sectionLabel}>Response Style</label>
            <p className={styles.sectionDescription}>
              Choose how the AI tutor responds to your questions
            </p>
            <div className={styles.modeGrid}>
              {RESPONSE_MODES.map((mode) => {
                const ModeIcon = mode.Icon;
                return (
                  <button
                    key={mode.value}
                    className={`${styles.modeButton} ${
                      settings.responseMode === mode.value ? styles.active : ''
                    }`}
                    onClick={() => handleModeChange(mode.value)}
                    aria-label={`Select ${mode.label} mode`}
                    aria-pressed={settings.responseMode === mode.value}
                  >
                    <ModeIcon className={styles.modeIcon} size={20} aria-hidden="true" />
                    <span className={styles.modeLabel}>{mode.label}</span>
                    <span className={styles.modeDescription}>
                      {mode.description}
                    </span>
                  </button>
                );
              })}
            </div>
          </div>

          {/* Explanation Depth */}
          <div className={styles.section}>
            <label className={styles.sectionLabel} htmlFor="depth-slider">
              Explanation Depth
            </label>
            <p className={styles.sectionDescription}>
              Adjust based on your experience level
            </p>
            <div className={styles.sliderContainer}>
              <div className={styles.sliderLabels}>
                <span className={styles.sliderLabelStart}>Beginner</span>
                <span className={styles.sliderLabelCurrent}>
                  {DEPTH_LABELS[settings.explanationDepth]}
                </span>
                <span className={styles.sliderLabelEnd}>Expert</span>
              </div>
              <input
                id="depth-slider"
                type="range"
                min="1"
                max="5"
                step="1"
                value={settings.explanationDepth}
                onChange={handleDepthChange}
                className={styles.slider}
                aria-label="Explanation depth slider"
                aria-valuemin={1}
                aria-valuemax={5}
                aria-valuenow={settings.explanationDepth}
                aria-valuetext={DEPTH_LABELS[settings.explanationDepth]}
              />
              <div className={styles.sliderTicks}>
                {[1, 2, 3, 4, 5].map((tick) => (
                  <span
                    key={tick}
                    className={`${styles.sliderTick} ${
                      settings.explanationDepth === tick ? styles.active : ''
                    }`}
                    aria-hidden="true"
                  />
                ))}
              </div>
            </div>
          </div>

          {/* Language Style */}
          <div className={styles.section}>
            <label className={styles.sectionLabel}>Language Style</label>
            <p className={styles.sectionDescription}>
              Choose the tone of explanations
            </p>
            <div className={styles.styleGrid}>
              {LANGUAGE_STYLES.map((style) => (
                <button
                  key={style.value}
                  className={`${styles.styleButton} ${
                    settings.languageStyle === style.value ? styles.active : ''
                  }`}
                  onClick={() => handleLanguageStyleChange(style.value)}
                  aria-label={`Select ${style.label} language style`}
                  aria-pressed={settings.languageStyle === style.value}
                >
                  <span className={styles.styleLabel}>{style.label}</span>
                  <span className={styles.styleDescription}>
                    {style.description}
                  </span>
                </button>
              ))}
            </div>
          </div>

          {/* Preferences */}
          <div className={styles.section}>
            <label className={styles.sectionLabel}>Preferences</label>
            <div className={styles.checkboxGroup}>
              <label className={styles.checkbox}>
                <input
                  type="checkbox"
                  checked={settings.includeCodeExamples}
                  onChange={handleCodeExamplesChange}
                  aria-label="Include code examples in responses"
                />
                <span className={styles.checkboxLabel}>
                  <span className={styles.checkboxIcon} aria-hidden="true">
                    💻
                  </span>
                  Include code examples
                </span>
                <span className={styles.checkboxDescription}>
                  Show practical code snippets when relevant
                </span>
              </label>

              <label className={styles.checkbox}>
                <input
                  type="checkbox"
                  checked={settings.includeVisuals}
                  onChange={handleVisualsChange}
                  aria-label="Include visual diagrams in responses"
                />
                <span className={styles.checkboxLabel}>
                  <span className={styles.checkboxIcon} aria-hidden="true">
                    📊
                  </span>
                  Include visual diagrams
                </span>
                <span className={styles.checkboxDescription}>
                  Generate Mermaid diagrams for complex concepts
                </span>
              </label>
            </div>
          </div>
        </div>

        {/* Footer */}
        <div className={styles.footer}>
          <button className={styles.resetButton} onClick={handleReset}>
            Reset to Defaults
          </button>
          <button className={styles.saveButton} onClick={onClose}>
            Done
          </button>
        </div>
      </div>
    </div>
  );
}
