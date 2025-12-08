import React, { useState, useEffect } from "react";
import { ChevronDown, ChevronRight, Check, X, Award } from "lucide-react";
import MultipleChoice from "./MultipleChoice";
import styles from "./QuizAccordion.module.css";

interface QuizQuestion {
  quizId: string;
  question: string;
  options: string[];
  correctAnswer: number;
  explanation: string;
  hint?: string;
  difficulty?: "easy" | "medium" | "hard";
}

interface QuizAccordionProps {
  sectionId: string;
  title?: string;
  questions: QuizQuestion[];
}

export default function QuizAccordion({
  sectionId,
  title = "Knowledge Check",
  questions,
}: QuizAccordionProps) {
  // Track which question is currently expanded
  const [expandedQuizId, setExpandedQuizId] = useState<string | null>(null);

  // Track completion status from localStorage
  const [completionStatus, setCompletionStatus] = useState<
    Record<string, boolean>
  >({});

  useEffect(() => {
    // Load completion status from localStorage
    const loadCompletionStatus = () => {
      const status: Record<string, boolean> = {};
      questions.forEach((q) => {
        const saved = localStorage.getItem(`physicalai_quiz_${q.quizId}`);
        if (saved) {
          try {
            const data = JSON.parse(saved);
            status[q.quizId] = data.correct === true;
          } catch (e) {
            status[q.quizId] = false;
          }
        } else {
          status[q.quizId] = false;
        }
      });
      setCompletionStatus(status);
    };

    loadCompletionStatus();

    // Listen for storage events to update completion in real-time
    const handleStorageChange = () => {
      loadCompletionStatus();
    };

    window.addEventListener("storage", handleStorageChange);
    // Custom event for same-tab updates
    window.addEventListener("quizCompleted", handleStorageChange);

    return () => {
      window.removeEventListener("storage", handleStorageChange);
      window.removeEventListener("quizCompleted", handleStorageChange);
    };
  }, [questions]);

  const toggleQuiz = (quizId: string) => {
    setExpandedQuizId(expandedQuizId === quizId ? null : quizId);
  };

  // Calculate progress
  const completedCount = Object.values(completionStatus).filter(Boolean).length;
  const totalCount = questions.length;
  const progressPercent = Math.round((completedCount / totalCount) * 100);

  return (
    <div className={styles.quizAccordion} data-section-id={sectionId}>
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerLeft}>
          <Award className={styles.icon} size={24} />
          <div className={styles.headerText}>
            <h3 className={styles.title}>{title}</h3>
            <p className={styles.subtitle}>
              Progress: {completedCount}/{totalCount} completed
            </p>
          </div>
        </div>
        <div className={styles.progressBadge}>{progressPercent}%</div>
      </div>

      {/* Progress Bar */}
      <div className={styles.progressBarContainer}>
        <div
          className={styles.progressBar}
          style={{ width: `${progressPercent}%` }}
        />
      </div>

      {/* Questions */}
      <div className={styles.questionsList}>
        {questions.map((q, index) => {
          const isExpanded = expandedQuizId === q.quizId;
          const isCompleted = completionStatus[q.quizId];

          return (
            <div
              key={q.quizId}
              className={`${styles.questionItem} ${
                isExpanded ? styles.expanded : ""
              }`}
            >
              {/* Question Header */}
              <button
                className={styles.questionHeader}
                onClick={() => toggleQuiz(q.quizId)}
                aria-expanded={isExpanded}
                aria-controls={`quiz-${q.quizId}`}
              >
                <div className={styles.questionHeaderLeft}>
                  {isExpanded ? (
                    <ChevronDown className={styles.chevron} size={20} />
                  ) : (
                    <ChevronRight className={styles.chevron} size={20} />
                  )}
                  <span className={styles.questionNumber}>
                    Question {index + 1}
                  </span>
                  <span className={styles.questionPreview}>{q.question}</span>
                </div>
                <div className={styles.questionHeaderRight}>
                  {isCompleted ? (
                    <div
                      className={styles.completionBadge}
                      data-status="completed"
                    >
                      <Check size={16} strokeWidth={2.5} />
                      <span>Completed</span>
                    </div>
                  ) : (
                    <div
                      className={styles.completionBadge}
                      data-status="pending"
                    >
                      <span>Not attempted</span>
                    </div>
                  )}
                </div>
              </button>

              {/* Question Content */}
              {isExpanded && (
                <div id={`quiz-${q.quizId}`} className={styles.questionContent}>
                  <MultipleChoice
                    quizId={q.quizId}
                    question={q.question}
                    options={q.options}
                    correctAnswer={q.correctAnswer}
                    explanation={q.explanation}
                    hint={q.hint}
                    difficulty={q.difficulty}
                  />
                </div>
              )}
            </div>
          );
        })}
      </div>

      {/* Summary */}
      {completedCount === totalCount && (
        <div className={styles.completionMessage}>
          <Award className={styles.completionIcon} size={24} />
          <span>
            🎉 Excellent work! You've completed all questions in this section.
          </span>
        </div>
      )}
    </div>
  );
}
