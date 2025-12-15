/**
 * ChatWidget Component - Modernized Professional UI
 * RAG-powered Q&A interface with Lucide icons
 */

import React, { useState, useRef, useEffect } from "react";
import ExecutionEnvironment from "@docusaurus/ExecutionEnvironment";
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

// Backend API URL - defaults to localhost if not deployed
const API_URL =
  process.env.NODE_ENV === "production"
    ? process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000/api/v1"
    : "http://localhost:8000/api/v1";

export default function ChatWidget() {
  const [isMounted, setIsMounted] = useState(false);
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
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Prevent SSR issues - only render on client
  useEffect(() => {
    setIsMounted(true);
  }, []);

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

  // Don't render until mounted on client-side
  if (!isMounted || !ExecutionEnvironment.canUseDOM) {
    return null;
  }

  return (
    <>
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
