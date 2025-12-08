import React from "react";
import {
  FlaskConical,
  Clock,
  TrendingUp,
  CheckCircle,
  ExternalLink,
} from "lucide-react";
import styles from "./LabCallout.module.css";

interface LabCalloutProps {
  labId: string;
  title: string;
  difficulty: "Beginner" | "Intermediate" | "Advanced";
  duration: string;
  prerequisites: string[];
  description: string;
  link: string;
}

export default function LabCallout({
  labId,
  title,
  difficulty,
  duration,
  prerequisites,
  description,
  link,
}: LabCalloutProps) {
  const getDifficultyColor = () => {
    switch (difficulty) {
      case "Beginner":
        return styles.difficultyBeginner;
      case "Intermediate":
        return styles.difficultyIntermediate;
      case "Advanced":
        return styles.difficultyAdvanced;
      default:
        return styles.difficultyBeginner;
    }
  };

  const getDifficultyIcon = () => {
    switch (difficulty) {
      case "Beginner":
        return "🟢";
      case "Intermediate":
        return "🟡";
      case "Advanced":
        return "🔴";
      default:
        return "🟢";
    }
  };

  return (
    <div className={styles.labCallout} data-lab-id={labId}>
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerLabel}>
          <FlaskConical
            className={styles.flaskIcon}
            size={20}
            strokeWidth={2.5}
          />
          <span>Try This Lab</span>
        </div>
      </div>

      {/* Title */}
      <h3 className={styles.title}>{title}</h3>

      {/* Description */}
      <p className={styles.description}>{description}</p>

      {/* Meta Info */}
      <div className={styles.metaInfo}>
        <div className={styles.metaItem}>
          <Clock size={16} />
          <span>{duration}</span>
        </div>
        <div className={`${styles.metaItem} ${getDifficultyColor()}`}>
          <TrendingUp size={16} />
          <span>
            {getDifficultyIcon()} {difficulty}
          </span>
        </div>
      </div>

      {/* Prerequisites */}
      {prerequisites.length > 0 && (
        <div className={styles.prerequisites}>
          <div className={styles.prerequisitesHeader}>
            <CheckCircle size={16} />
            <span>Prerequisites:</span>
          </div>
          <ul className={styles.prerequisitesList}>
            {prerequisites.map((prereq, index) => (
              <li key={index} className={styles.prerequisiteItem}>
                {prereq}
              </li>
            ))}
          </ul>
        </div>
      )}

      {/* Action Button */}
      <a
        href={`/physical-ai-and-robotics-book${link}`}
        className={styles.startButton}
        aria-label={`Start ${title}`}
      >
        <span>Start Lab</span>
        <ExternalLink size={18} strokeWidth={2.5} />
      </a>
    </div>
  );
}
