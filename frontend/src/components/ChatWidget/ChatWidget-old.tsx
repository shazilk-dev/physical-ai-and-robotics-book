/**
 * ChatWidget Component
 * Floating chat button that expands into a RAG-powered Q&A interface
 */

import React, { useState, useRef, useEffect } from "react";
import styles from "./ChatWidget.module.css";

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
}

interface Message {
  type: "user" | "assistant" | "error";
  content: string;
  sources?: Source[];
  citations?: string[];
  timestamp: Date;
}

const API_URL =
  process.env.NODE_ENV === "production"
    ? process.env.NEXT_PUBLIC_API_URL ||
      "https://your-backend.up.railway.app/api/v1"
    : "http://localhost:8000/api/v1";

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      type: "assistant",
      content:
        "👋 Hi! I'm your Physical AI learning assistant. Ask me anything about ROS 2, URDF, sensors, or robotics!",
      timestamp: new Date(),
    },
  ]);
  const [inputValue, setInputValue] = useState("");
  const [isLoading, setIsLoading] = useState(false);
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
      const response = await fetch(`${API_URL}/query`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          question: userMessage.content,
          num_results: 5,
        }),
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
    } catch (error) {
      console.error("Query error:", error);

      const errorMessage: Message = {
        type: "error",
        content: `⚠️ Sorry, I couldn't process your question. ${error.message}. Make sure the backend server is running.`,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const scrollToSection = (filePath: string, section: string) => {
    // Convert file path to URL
    // Example: "frontend/docs/module-01-ros2/ros2-fundamentals/1.1.1-architecture.md"
    // -> "/docs/module-01-ros2/ros2-fundamentals/1.1.1-architecture"

    const docPath = filePath
      .replace("frontend/docs/", "/docs/")
      .replace(".md", "")
      .replace(/\\/g, "/");

    // Navigate to the page
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
          {!isUser && <div className={styles.avatar}>🤖</div>}

          <div className={styles.messageBody}>
            <div className={styles.messageText}>{message.content}</div>

            {message.citations && message.citations.length > 0 && (
              <div className={styles.citations}>
                <div className={styles.citationsLabel}>📚 Sources:</div>
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
                        title={`Open ${citation} (score: ${source?.score.toFixed(
                          2
                        )})`}
                      >
                        {citation}
                      </button>
                    );
                  })}
                </div>
              </div>
            )}
          </div>

          {isUser && <div className={styles.avatar}>👤</div>}
        </div>
      </div>
    );
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.open : ""}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? "✕" : "💬"}
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <div className={styles.headerTitle}>
              <span className={styles.headerIcon}>🤖</span>
              <span>AI Learning Assistant</span>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          <div className={styles.messagesContainer}>
            {messages.map((message, index) => renderMessage(message, index))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.messageContent}>
                  <div className={styles.avatar}>🤖</div>
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
              {isLoading ? "⏳" : "📤"}
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
