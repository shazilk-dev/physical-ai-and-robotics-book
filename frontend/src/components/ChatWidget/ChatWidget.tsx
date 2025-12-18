/**
 * ChatWidget Component - Modernized Professional UI
 * RAG-powered Q&A interface with Lucide icons
 * Enhanced with text selection and contextual queries
 */

import React, { useState, useRef, useEffect } from "react";
import {
  MessageCircle,
  X,
  Send,
  Bot,
  User,
  BookOpen,
  AlertCircle,
  Loader2,
  Settings,
  Sparkles,
} from "lucide-react";
import styles from "./ChatWidget.module.css";
import SelectionMenu, { SelectionContext } from "../SelectionMenu/SelectionMenu";
import ChatSettingsPanel from "./ChatSettingsPanel";
import {
  ChatSettings,
  DEFAULT_SETTINGS,
  LLMProvider,
  DEFAULT_PROVIDER,
  LLM_PROVIDERS,
} from "./types";

interface Source {
  content: string;
  module: string;
  chapter: string;
  section: string;
  section_title: string;
  file_path: string;
  score: number;
}

interface QueryResponse {
  answer: string;
  sources: Source[];
  citations: string[];
  model?: string;
  action?: string; // Track which action was used
}

interface ContextualQueryRequest {
  question: string;
  selected_text?: string;
  action?: string;
  num_results?: number;
  settings?: ChatSettings; // Include user preferences
  provider?: string; // LLM provider override
}

interface Message {
  type: "user" | "assistant" | "error";
  content: string;
  sources?: Source[];
  citations?: string[];
  timestamp: Date;
  action?: "explain" | "simplify" | "example" | "quiz"; // Track which action was used
  selectedText?: string; // Store the selected text for context
}

// Backend API URL - detect environment safely in browser
const getApiUrl = () => {
  // Check if we have a custom API URL set (from environment variables)
  if (typeof window !== 'undefined' && (window as any).docusaurus?.siteConfig?.customFields?.apiUrl) {
    return (window as any).docusaurus.siteConfig.customFields.apiUrl;
  }

  // Check if running on production domain (Vercel)
  if (typeof window !== 'undefined' &&
      (window.location.hostname.includes('vercel.app') ||
       window.location.hostname.includes('physical-ai-robotics-book'))) {
    return 'https://physical-ai-and-robotics-book.onrender.com/api/v1';
  }

  // Default to localhost for development
  return 'http://localhost:8000/api/v1';
};

const API_URL = getApiUrl();

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      type: "assistant",
      content:
        "Hi! I'm your Physical AI learning assistant. Ask me anything about ROS 2, URDF, sensors, or robotics concepts from the book.",
      timestamp: new Date(),
    },
  ]);
  const [inputValue, setInputValue] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [loadingMessage, setLoadingMessage] = useState<string>("Thinking...");
  const [selectionContext, setSelectionContext] = useState<SelectionContext | null>(null);
  const [showSettings, setShowSettings] = useState(false);
  const [settings, setSettings] = useState<ChatSettings>(DEFAULT_SETTINGS);
  const [provider, setProvider] = useState<LLMProvider>(DEFAULT_PROVIDER);
  const [showProviderMenu, setShowProviderMenu] = useState(false);
  const [viewportHeight, setViewportHeight] = useState(typeof window !== 'undefined' ? window.innerHeight : 0);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen) {
      inputRef.current?.focus();
    }
  }, [isOpen]);

  // Load settings from localStorage on mount
  useEffect(() => {
    try {
      const savedSettings = localStorage.getItem('chatSettings');
      if (savedSettings) {
        setSettings(JSON.parse(savedSettings));
      }
    } catch (error) {
      console.error('Failed to load chat settings:', error);
    }
  }, []);

  // Save settings to localStorage whenever they change
  useEffect(() => {
    try {
      localStorage.setItem('chatSettings', JSON.stringify(settings));
    } catch (error) {
      console.error('Failed to save chat settings:', error);
    }
  }, [settings]);

  // Load provider from localStorage on mount
  useEffect(() => {
    try {
      const savedProvider = localStorage.getItem('chatProvider');
      if (savedProvider && (savedProvider === 'openai' || savedProvider === 'gemini' || savedProvider === 'qwen')) {
        setProvider(savedProvider as LLMProvider);
      }
    } catch (error) {
      console.error('Failed to load provider:', error);
    }
  }, []);

  // Save provider to localStorage whenever it changes
  useEffect(() => {
    try {
      localStorage.setItem('chatProvider', provider);
    } catch (error) {
      console.error('Failed to save provider:', error);
    }
  }, [provider]);

  // Handle virtual keyboard on mobile - prevents keyboard from hiding input
  useEffect(() => {
    // Only run in browser
    if (typeof window === 'undefined') return;

    const handleResize = () => {
      // Use visualViewport API for modern browsers (iOS 15.4+, Android Chrome 108+)
      if (window.visualViewport) {
        setViewportHeight(window.visualViewport.height);
        // Set CSS variable for fallback in CSS
        document.documentElement.style.setProperty(
          '--vh',
          `${window.visualViewport.height * 0.01}px`
        );
      } else {
        // Fallback for older browsers
        setViewportHeight(window.innerHeight);
        document.documentElement.style.setProperty(
          '--vh',
          `${window.innerHeight * 0.01}px`
        );
      }
    };

    // Initial setup
    handleResize();

    // Listen to viewport changes (keyboard show/hide)
    if (window.visualViewport) {
      window.visualViewport.addEventListener('resize', handleResize);
      window.visualViewport.addEventListener('scroll', handleResize);
    } else {
      window.addEventListener('resize', handleResize);
    }

    // Cleanup listeners
    return () => {
      if (window.visualViewport) {
        window.visualViewport.removeEventListener('resize', handleResize);
        window.visualViewport.removeEventListener('scroll', handleResize);
      } else {
        window.removeEventListener('resize', handleResize);
      }
    };
  }, []);

  // Handle text selection actions
  const handleSelectionAction = (context: SelectionContext) => {
    // Open chat if closed
    if (!isOpen) {
      setIsOpen(true);
    }

    // Generate appropriate question based on action
    const prompts = {
      explain: `Explain this concept: "${context.text}"`,
      simplify: `Explain this in simpler terms: "${context.text}"`,
      example: `Give me a practical example of: "${context.text}"`,
      quiz: `Quiz me on this concept: "${context.text}"`,
    };

    // Set the input with the generated prompt
    setInputValue(prompts[context.action]);
    setSelectionContext(context);

    // Auto-submit the query after a brief delay (to show the prompt)
    setTimeout(() => {
      // Trigger form submission programmatically
      const form = document.querySelector(`.${styles.inputForm}`) as HTMLFormElement;
      if (form) {
        form.dispatchEvent(new Event('submit', { cancelable: true, bubbles: true }));
      }
    }, 500);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      type: "user",
      content: inputValue.trim(),
      timestamp: new Date(),
      action: selectionContext?.action,
      selectedText: selectionContext?.text,
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue("");
    setIsLoading(true);

    // Set contextual loading message
    const loadingMessages = {
      explain: "Preparing detailed explanation...",
      simplify: "Simplifying the concept...",
      example: "Generating code example...",
      quiz: "Creating quiz questions...",
    };
    setLoadingMessage(
      selectionContext?.action
        ? loadingMessages[selectionContext.action]
        : "Searching textbook..."
    );

    try {
      // Build request payload with optional selection context and settings
      const requestPayload: ContextualQueryRequest = {
        question: userMessage.content,
        num_results: 5,
        settings: settings, // Include user preferences
        provider: provider, // Include selected provider
      };

      // Add selection context if available
      if (selectionContext) {
        requestPayload.selected_text = selectionContext.text;
        requestPayload.action = selectionContext.action;
      }

      const response = await fetch(`${API_URL}/query/contextual`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(requestPayload),
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data: QueryResponse = await response.json();

      const assistantMessage: Message = {
        type: "assistant",
        content: data.answer,
        sources: data.sources,
        citations: data.citations,
        timestamp: new Date(),
        action: data.action || selectionContext?.action, // Store action from response or context
      };

      setMessages((prev) => [...prev, assistantMessage]);

      // Clear selection context after successful query
      setSelectionContext(null);
    } catch (error) {
      console.error("Query error:", error);

      // Generate user-friendly error message
      let errorContent = "Sorry, I encountered an issue. ";

      if (error.message.includes("Failed to fetch") || error.message.includes("NetworkError")) {
        errorContent += "I couldn't connect to the backend server. Please check your internet connection or try again later.";
      } else if (error.message.includes("500")) {
        errorContent += "The server encountered an error. Our team has been notified. Please try again in a moment.";
      } else if (error.message.includes("404")) {
        errorContent += "The requested endpoint wasn't found. This might be a deployment issue.";
      } else {
        errorContent += `${error.message}. Please try rephrasing your question or try again.`;
      }

      const errorMessage: Message = {
        type: "error",
        content: errorContent,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setLoadingMessage("Thinking..."); // Reset loading message
    }
  };

  const scrollToSection = (filePath: string, section: string) => {
    const docPath = filePath
      .replace("frontend/docs/", "/docs/")
      .replace(".md", "")
      .replace(/\\/g, "/");

    window.location.href = docPath;
  };

  // Get action badge info with Lucide icons
  const getActionBadge = (action: Message["action"]) => {
    if (!action) return null;

    const badges = {
      explain: {
        Icon: BookOpen,
        label: "Detailed Explanation",
        color: "#667eea"
      },
      simplify: {
        Icon: Sparkles,
        label: "Simplified",
        color: "#48bb78"
      },
      example: {
        Icon: Bot,
        label: "Code Example",
        color: "#f6ad55"
      },
      quiz: {
        Icon: MessageCircle,
        label: "Quiz Mode",
        color: "#ed64a6"
      },
    };

    return badges[action];
  };

  const renderMessage = (message: Message, index: number) => {
    const isUser = message.type === "user";
    const isError = message.type === "error";
    const actionBadge = getActionBadge(message.action);

    return (
      <div
        key={index}
        className={`${styles.message} ${
          isUser ? styles.userMessage : styles.assistantMessage
        } ${isError ? styles.errorMessage : ""}`}
      >
        <div className={styles.messageContent}>
          {!isUser && (
            <div className={styles.avatarWrapper}>
              {isError ? (
                <AlertCircle className={styles.avatarIcon} size={20} />
              ) : (
                <Bot className={styles.avatarIcon} size={20} />
              )}
            </div>
          )}

          <div className={styles.messageBody}>
            {/* Show badges for assistant messages */}
            {!isUser && (
              <div className={styles.badgesContainer}>
                {/* Action badge */}
                {actionBadge && (
                  <div
                    className={styles.actionBadge}
                    style={{ backgroundColor: actionBadge.color }}
                  >
                    <actionBadge.Icon className={styles.badgeIcon} size={14} />
                    <span className={styles.badgeLabel}>{actionBadge.label}</span>
                  </div>
                )}
                {/* Provider badge */}
                <div
                  className={styles.providerBadge}
                  style={{
                    backgroundColor: LLM_PROVIDERS.find(p => p.value === provider)?.color + '15',
                    borderColor: LLM_PROVIDERS.find(p => p.value === provider)?.color,
                  }}
                >
                  <span className={styles.providerBadgeIcon}>
                    {LLM_PROVIDERS.find(p => p.value === provider)?.icon}
                  </span>
                  <span className={styles.providerBadgeLabel}>
                    {LLM_PROVIDERS.find(p => p.value === provider)?.name}
                  </span>
                </div>
              </div>
            )}

            {/* Show selected text for user messages */}
            {isUser && message.selectedText && (
              <div className={styles.selectedTextIndicator}>
                <span className={styles.selectedTextLabel}>Selected:</span>
                <span className={styles.selectedTextContent}>
                  "{message.selectedText.length > 80
                    ? message.selectedText.substring(0, 77) + "..."
                    : message.selectedText}"
                </span>
              </div>
            )}

            <div className={styles.messageText}>{message.content}</div>

            {message.citations && message.citations.length > 0 && (
              <div className={styles.citations}>
                <div className={styles.citationsLabel}>
                  <BookOpen size={14} />
                  <span>Sources</span>
                </div>
                <div className={styles.citationsList}>
                  {message.citations.map((citation, i) => {
                    const source = message.sources?.[i];
                    return (
                      <button
                        key={i}
                        className={styles.citationButton}
                        onClick={() =>
                          source && scrollToSection(source.file_path, citation)
                        }
                        title={`Open ${citation} (relevance: ${(
                          (source?.score || 0) * 100
                        ).toFixed(0)}%)`}
                      >
                        {citation}
                      </button>
                    );
                  })}
                </div>
              </div>
            )}
          </div>

          {isUser && (
            <div className={styles.avatarWrapper}>
              <User className={styles.avatarIcon} size={20} />
            </div>
          )}
        </div>
      </div>
    );
  };

  return (
    <>
      {/* Text Selection Menu */}
      <SelectionMenu onAction={handleSelectionAction} />

      {/* Settings Panel */}
      {showSettings && (
        <ChatSettingsPanel
          settings={settings}
          onUpdate={setSettings}
          onClose={() => setShowSettings(false)}
        />
      )}

      {/* Floating Button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.open : ""}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? (
          <X size={24} strokeWidth={2.5} />
        ) : (
          <MessageCircle size={24} strokeWidth={2.5} />
        )}
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <div className={styles.headerTitle}>
              <Bot size={24} strokeWidth={2} />
              <div>
                <div className={styles.headerTitleText}>AI Assistant</div>
                <div className={styles.headerSubtitle}>Physical AI Book</div>
              </div>
            </div>
            <div className={styles.headerButtons}>
              {/* Provider Selector */}
              <div className={styles.providerSelector}>
                <button
                  className={styles.providerButton}
                  onClick={() => setShowProviderMenu(!showProviderMenu)}
                  aria-label="Select AI provider"
                  title={`Current: ${LLM_PROVIDERS.find(p => p.value === provider)?.name}`}
                >
                  {(() => {
                    const currentProvider = LLM_PROVIDERS.find(p => p.value === provider);
                    const IconComponent = currentProvider?.Icon;
                    return IconComponent ? <IconComponent size={16} /> : null;
                  })()}
                  <span style={{ fontSize: '13px', fontWeight: 500 }}>
                    {LLM_PROVIDERS.find(p => p.value === provider)?.name}
                  </span>
                </button>

                {showProviderMenu && (
                  <div className={styles.providerMenu}>
                    <div className={styles.providerMenuHeader}>
                      <Sparkles size={12} />
                      AI Provider
                    </div>
                    {LLM_PROVIDERS.map((p) => {
                      const ProviderIcon = p.Icon;
                      return (
                        <button
                          key={p.value}
                          className={`${styles.providerOption} ${
                            provider === p.value ? styles.active : ''
                          }`}
                          onClick={() => {
                            setProvider(p.value);
                            setShowProviderMenu(false);
                          }}
                        >
                          <div className={styles.providerIcon}>
                            <ProviderIcon size={18} />
                          </div>
                          <div className={styles.providerInfo}>
                            <div className={styles.providerName}>{p.name}</div>
                            <div className={styles.providerDesc}>{p.description}</div>
                          </div>
                          {provider === p.value && (
                            <span className={styles.checkmark}>✓</span>
                          )}
                        </button>
                      );
                    })}
                  </div>
                )}
              </div>

              <button
                className={styles.settingsButton}
                onClick={() => setShowSettings(true)}
                aria-label="Open settings"
                title="Customize response style"
              >
                <Settings size={20} />
              </button>
              <button
                className={styles.closeButton}
                onClick={() => setIsOpen(false)}
                aria-label="Close chat"
              >
                <X size={20} />
              </button>
            </div>
          </div>

          <div className={styles.messagesContainer}>
            {messages.map((message, index) => renderMessage(message, index))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.messageContent}>
                  <div className={styles.avatarWrapper}>
                    <Loader2
                      className={`${styles.avatarIcon} ${styles.spinning}`}
                      size={20}
                    />
                  </div>
                  <div className={styles.messageBody}>
                    <div className={styles.loadingText}>
                      {loadingMessage}
                    </div>
                    <div className={styles.loadingDots}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about ROS 2, URDF, sensors..."
              className={styles.input}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
              aria-label="Send message"
            >
              <Send size={18} />
            </button>
          </form>

          <div className={styles.footer}>
            <span className={styles.footerText}>
              Powered by GPT-4o-mini & Qdrant
            </span>
          </div>
        </div>
      )}
    </>
  );
}
