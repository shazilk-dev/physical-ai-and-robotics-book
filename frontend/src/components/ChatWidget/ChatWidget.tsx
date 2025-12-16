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
} from "lucide-react";
import styles from "./ChatWidget.module.css";
import SelectionMenu, { SelectionContext } from "../SelectionMenu/SelectionMenu";

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
}

interface Message {
  type: "user" | "assistant" | "error";
  content: string;
  sources?: Source[];
  citations?: string[];
  timestamp: Date;
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
  const [selectionContext, setSelectionContext] = useState<SelectionContext | null>(null);
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
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue("");
    setIsLoading(true);

    try {
      // Build request payload with optional selection context
      const requestPayload: ContextualQueryRequest = {
        question: userMessage.content,
        num_results: 5,
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
      };

      setMessages((prev) => [...prev, assistantMessage]);

      // Clear selection context after successful query
      setSelectionContext(null);
    } catch (error) {
      console.error("Query error:", error);

      const errorMessage: Message = {
        type: "error",
        content: `Sorry, I couldn't process your question. ${error.message}. Make sure the backend server is running.`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const scrollToSection = (filePath: string, section: string) => {
    const docPath = filePath
      .replace("frontend/docs/", "/docs/")
      .replace(".md", "")
      .replace(/\\/g, "/");

    window.location.href = docPath;
  };

  const renderMessage = (message: Message, index: number) => {
    const isUser = message.type === "user";
    const isError = message.type === "error";

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
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              <X size={20} />
            </button>
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
