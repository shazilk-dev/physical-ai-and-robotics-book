/**
 * Type Definitions for ChatWidget Settings
 * Supports personalized conversation modes and user preferences
 */

import {
  LucideIcon,
  Zap,
  BookOpen,
  GraduationCap,
  MessageSquare,
  Bot,
  Sparkles,
  Star,
} from 'lucide-react';

/**
 * Response mode determines the style and length of AI responses
 */
export type ResponseMode = 'quick' | 'detailed' | 'tutorial' | 'socratic';

/**
 * Language style affects the tone of AI responses
 */
export type LanguageStyle = 'casual' | 'formal' | 'technical';

/**
 * LLM Provider - which AI service to use
 */
export type LLMProvider = 'openai' | 'gemini' | 'qwen';

/**
 * Chat settings interface - user preferences for AI responses
 */
export interface ChatSettings {
  /** Response style mode */
  responseMode: ResponseMode;
  /** Explanation depth (1=Beginner, 5=Expert) */
  explanationDepth: number;
  /** Whether to include code examples in responses */
  includeCodeExamples: boolean;
  /** Whether to include visual diagrams when relevant */
  includeVisuals: boolean;
  /** Language/tone style */
  languageStyle: LanguageStyle;
}

/**
 * Default chat settings
 */
export const DEFAULT_SETTINGS: ChatSettings = {
  responseMode: 'detailed',
  explanationDepth: 3,
  includeCodeExamples: true,
  includeVisuals: true,
  languageStyle: 'casual',
};

/**
 * Response mode metadata for UI display
 */
export interface ResponseModeInfo {
  value: ResponseMode;
  Icon: LucideIcon;
  label: string;
  description: string;
}

/**
 * Available response modes with metadata
 */
export const RESPONSE_MODES: ResponseModeInfo[] = [
  {
    value: 'quick',
    Icon: Zap,
    label: 'Quick',
    description: 'Concise answers (1-2 paragraphs)',
  },
  {
    value: 'detailed',
    Icon: BookOpen,
    label: 'Detailed',
    description: 'Comprehensive explanations',
  },
  {
    value: 'tutorial',
    Icon: GraduationCap,
    label: 'Tutorial',
    description: 'Step-by-step guidance',
  },
  {
    value: 'socratic',
    Icon: MessageSquare,
    label: 'Socratic',
    description: 'Guided questions to help you learn',
  },
];

/**
 * Depth level labels for explanation depth slider
 */
export const DEPTH_LABELS: Record<number, string> = {
  1: 'Beginner',
  2: 'Basic',
  3: 'Intermediate',
  4: 'Advanced',
  5: 'Expert',
};

/**
 * Language style options with descriptions
 */
export interface LanguageStyleInfo {
  value: LanguageStyle;
  label: string;
  description: string;
}

export const LANGUAGE_STYLES: LanguageStyleInfo[] = [
  {
    value: 'casual',
    label: 'Casual',
    description: 'Friendly, conversational tone',
  },
  {
    value: 'formal',
    label: 'Formal',
    description: 'Professional, academic style',
  },
  {
    value: 'technical',
    label: 'Technical',
    description: 'Precise technical terminology',
  },
];

/**
 * LLM Provider information for UI display
 */
export interface LLMProviderInfo {
  value: LLMProvider;
  name: string;
  Icon: LucideIcon;
  description: string;
  color: string;
}

/**
 * Available LLM providers with metadata
 */
export const LLM_PROVIDERS: LLMProviderInfo[] = [
  {
    value: 'openai',
    name: 'OpenAI',
    Icon: Bot,
    description: 'GPT-4o-mini (Best quality, paid)',
    color: '#10a37f',
  },
  {
    value: 'gemini',
    name: 'Gemini',
    Icon: Sparkles,
    description: 'Google Gemini 1.5 Flash (Free)',
    color: '#4285f4',
  },
  {
    value: 'qwen',
    name: 'Qwen',
    Icon: Star,
    description: 'Alibaba Qwen (Affordable)',
    color: '#ff6a00',
  },
];

/**
 * Default LLM provider
 */
export const DEFAULT_PROVIDER: LLMProvider = 'openai';
