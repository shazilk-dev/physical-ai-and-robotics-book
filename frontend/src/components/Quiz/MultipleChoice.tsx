import React, { useState, useEffect } from "react";
import { Check, X, HelpCircle, RotateCcw, ChevronRight } from "lucide-react";
import styles from "./MultipleChoice.module.css";

interface MultipleChoiceProps {
  quizId: string;
  question: string;
  options: string[];
  correctAnswer: number;
  explanation: string;
  hint?: string;
  difficulty?: "easy" | "medium" | "hard";
}

interface QuizProgress {
  [quizId: string]: {
    completed: boolean;
    score: number;
    attempts: number;
    lastAttempt: string;
    selectedAnswer: number;
  };
}

export default function MultipleChoice({
  quizId,
  question,
  options,
  correctAnswer,
  explanation,
  hint,
  difficulty = "medium",
}: MultipleChoiceProps) {
  const [selectedOption, setSelectedOption] = useState<number | null>(null);
  const [submitted, setSubmitted] = useState(false);
  const [showHint, setShowHint] = useState(false);
  const [attempts, setAttempts] = useState(0);

  // Load previous progress from localStorage
  useEffect(() => {
    const savedProgress = localStorage.getItem("physicalai_quiz_progress");
    if (savedProgress) {
      try {
        const progress: QuizProgress = JSON.parse(savedProgress);
        if (progress[quizId]) {
          setSelectedOption(progress[quizId].selectedAnswer);
          setSubmitted(progress[quizId].completed);
          setAttempts(progress[quizId].attempts);
        }
      } catch (e) {
        console.error("Failed to load quiz progress:", e);
      }
    }
  }, [quizId]);

  const handleSubmit = () => {
    if (selectedOption === null) return;

    const isCorrect = selectedOption === correctAnswer;
    const newAttempts = attempts + 1;
    setSubmitted(true);
    setAttempts(newAttempts);

    // Save progress
    const savedProgress = localStorage.getItem("physicalai_quiz_progress");
    const progress: QuizProgress = savedProgress
      ? JSON.parse(savedProgress)
      : {};

    progress[quizId] = {
      completed: isCorrect,
      score: isCorrect ? 100 : 0,
      attempts: newAttempts,
      lastAttempt: new Date().toISOString(),
      selectedAnswer: selectedOption,
    };

    localStorage.setItem("physicalai_quiz_progress", JSON.stringify(progress));

    // Update overall progress stats
    updateGlobalProgress(isCorrect);
  };

  const updateGlobalProgress = (isCorrect: boolean) => {
    const statsKey = "physicalai_quiz_stats";
    const savedStats = localStorage.getItem(statsKey);
    const stats = savedStats
      ? JSON.parse(savedStats)
      : {
          totalQuizzes: 0,
          completedQuizzes: 0,
          totalScore: 0,
          lastUpdated: new Date().toISOString(),
        };

    if (isCorrect && !stats.completedQuizzes) {
      stats.completedQuizzes += 1;
    }
    stats.totalQuizzes = Math.max(stats.totalQuizzes, stats.completedQuizzes);
    stats.totalScore = isCorrect ? stats.totalScore + 100 : stats.totalScore;
    stats.lastUpdated = new Date().toISOString();

    localStorage.setItem(statsKey, JSON.stringify(stats));

    // Dispatch custom event to notify accordion of completion
    window.dispatchEvent(new Event("quizCompleted"));
  };

  const handleRetry = () => {
    setSelectedOption(null);
    setSubmitted(false);
    setShowHint(false);
  };

  const handleNext = () => {
    // Scroll to next content section
    const nextElement = document.querySelector(
      `[data-quiz-id="${quizId}"]`
    )?.nextElementSibling;
    if (nextElement) {
      nextElement.scrollIntoView({ behavior: "smooth", block: "start" });
    }
  };

  const isCorrect = submitted && selectedOption === correctAnswer;
  const isIncorrect = submitted && selectedOption !== correctAnswer;

  const getDifficultyColor = () => {
    switch (difficulty) {
      case "easy":
        return styles.difficultyEasy;
      case "medium":
        return styles.difficultyMedium;
      case "hard":
        return styles.difficultyHard;
      default:
        return styles.difficultyMedium;
    }
  };

  return (
    <div className={styles.quizContainer} data-quiz-id={quizId}>
      {/* Header */}
      <div className={styles.quizHeader}>
        <div className={styles.quizLabel}>
          <span className={styles.quizIcon}>💡</span>
          <span>Knowledge Check</span>
        </div>
        <div className={`${styles.difficultyBadge} ${getDifficultyColor()}`}>
          {difficulty.charAt(0).toUpperCase() + difficulty.slice(1)}
        </div>
      </div>

      {/* Question */}
      <div className={styles.question}>{question}</div>

      {/* Options */}
      <div className={styles.optionsContainer}>
        {options.map((option, index) => {
          const isSelected = selectedOption === index;
          const isThisCorrect = index === correctAnswer;
          const showCorrectIndicator = submitted && isThisCorrect;
          const showIncorrectIndicator =
            submitted && isSelected && !isThisCorrect;

          return (
            <button
              key={index}
              className={`
                ${styles.option}
                ${isSelected ? styles.optionSelected : ""}
                ${showCorrectIndicator ? styles.optionCorrect : ""}
                ${showIncorrectIndicator ? styles.optionIncorrect : ""}
                ${submitted ? styles.optionDisabled : ""}
              `}
              onClick={() => !submitted && setSelectedOption(index)}
              disabled={submitted}
              aria-label={`Option ${index + 1}: ${option}`}
              aria-pressed={isSelected}
            >
              <span className={styles.optionLetter}>
                {String.fromCharCode(65 + index)}
              </span>
              <span className={styles.optionText}>{option}</span>
              {showCorrectIndicator && (
                <Check
                  className={styles.correctIcon}
                  size={20}
                  strokeWidth={2.5}
                />
              )}
              {showIncorrectIndicator && (
                <X
                  className={styles.incorrectIcon}
                  size={20}
                  strokeWidth={2.5}
                />
              )}
            </button>
          );
        })}
      </div>

      {/* Hint Section */}
      {hint && !submitted && (
        <div className={styles.hintSection}>
          {!showHint ? (
            <button
              className={styles.hintButton}
              onClick={() => setShowHint(true)}
              aria-label="Show hint"
            >
              <HelpCircle size={16} />
              <span>Need a hint?</span>
            </button>
          ) : (
            <div className={styles.hintBox}>
              <div className={styles.hintHeader}>
                <HelpCircle size={18} />
                <span>Hint</span>
              </div>
              <p className={styles.hintText}>{hint}</p>
            </div>
          )}
        </div>
      )}

      {/* Feedback Section */}
      {submitted && (
        <div
          className={`${styles.feedback} ${
            isCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect
          }`}
        >
          <div className={styles.feedbackHeader}>
            {isCorrect ? (
              <>
                <Check
                  className={styles.feedbackIcon}
                  size={24}
                  strokeWidth={2.5}
                />
                <span className={styles.feedbackTitle}>Correct!</span>
              </>
            ) : (
              <>
                <X
                  className={styles.feedbackIcon}
                  size={24}
                  strokeWidth={2.5}
                />
                <span className={styles.feedbackTitle}>Not quite right</span>
              </>
            )}
          </div>
          <p className={styles.feedbackText}>{explanation}</p>
          {!isCorrect && (
            <p className={styles.feedbackHint}>
              The correct answer is{" "}
              <strong>
                {String.fromCharCode(65 + correctAnswer)}:{" "}
                {options[correctAnswer]}
              </strong>
            </p>
          )}
        </div>
      )}

      {/* Action Buttons */}
      <div className={styles.actions}>
        {!submitted ? (
          <button
            className={styles.submitButton}
            onClick={handleSubmit}
            disabled={selectedOption === null}
            aria-label="Submit answer"
          >
            Submit Answer
          </button>
        ) : (
          <div className={styles.actionButtons}>
            {!isCorrect && (
              <button
                className={styles.retryButton}
                onClick={handleRetry}
                aria-label="Try again"
              >
                <RotateCcw size={18} />
                <span>Try Again</span>
              </button>
            )}
            <button
              className={styles.nextButton}
              onClick={handleNext}
              aria-label="Continue reading"
            >
              <span>Continue</span>
              <ChevronRight size={18} />
            </button>
          </div>
        )}
      </div>

      {/* Attempts Counter */}
      {attempts > 0 && (
        <div className={styles.attemptsCounter}>Attempts: {attempts}</div>
      )}
    </div>
  );
}
