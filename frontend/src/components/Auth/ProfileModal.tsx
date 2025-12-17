import React, { useEffect } from "react";
import { createPortal } from "react-dom";
import { User, Mail, Calendar, Code, Cpu, Wrench, Box } from "lucide-react";
import styles from "./ProfileModal.module.css";

interface User {
  id: string;
  email: string;
  name: string;
  emailVerified: boolean;
  createdAt: string;
  updatedAt: string;
  softwareBackground?: string;
  hardwareBackground?: string;
  pythonExperience?: string;
  ros2Experience?: string;
  roboticsExperience?: string;
}

interface ProfileModalProps {
  isOpen: boolean;
  onClose: () => void;
  user: User;
}

export default function ProfileModal({
  isOpen,
  onClose,
  user,
}: ProfileModalProps) {
  const [mounted, setMounted] = React.useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!isOpen || !mounted) return null;

  const handleBackdropClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
  };

  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString("en-US", {
      year: "numeric",
      month: "long",
      day: "numeric",
    });
  };

  const getExperienceBadge = (level?: string) => {
    if (!level) return null;

    const badges = {
      none: { label: "No Experience", color: "#94a3b8" },
      beginner: { label: "Beginner", color: "#3b82f6" },
      intermediate: { label: "Intermediate", color: "#10b981" },
      advanced: { label: "Advanced", color: "#8b5cf6" },
    };

    const badge = badges[level as keyof typeof badges] || badges.beginner;

    return (
      <span
        className={styles.badge}
        style={{ backgroundColor: badge.color }}
      >
        {badge.label}
      </span>
    );
  };

  const modalContent = (
    <div className={styles.backdrop} onClick={handleBackdropClick}>
      <div className={styles.modal}>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close"
        >
          ✕
        </button>

        <div className={styles.header}>
          <div className={styles.avatarLarge}>
            {user.name?.[0]?.toUpperCase() || "U"}
          </div>
          <h2>{user.name || "User"}</h2>
          <p className={styles.email}>
            <Mail size={14} />
            {user.email}
          </p>
        </div>

        <div className={styles.content}>
          <div className={styles.section}>
            <h3>
              <User size={18} />
              Account Information
            </h3>
            <div className={styles.infoGrid}>
              <div className={styles.infoItem}>
                <span className={styles.infoLabel}>Member Since</span>
                <span className={styles.infoValue}>
                  <Calendar size={14} />
                  {formatDate(user.createdAt)}
                </span>
              </div>
              <div className={styles.infoItem}>
                <span className={styles.infoLabel}>Email Status</span>
                <span className={styles.infoValue}>
                  {user.emailVerified ? (
                    <span className={styles.verified}>✓ Verified</span>
                  ) : (
                    <span className={styles.unverified}>Unverified</span>
                  )}
                </span>
              </div>
            </div>
          </div>

          <div className={styles.section}>
            <h3>
              <Code size={18} />
              Experience & Background
            </h3>
            <div className={styles.experienceGrid}>
              {user.softwareBackground && (
                <div className={styles.experienceItem}>
                  <span className={styles.experienceLabel}>
                    <Cpu size={16} />
                    Software
                  </span>
                  {getExperienceBadge(user.softwareBackground)}
                </div>
              )}
              {user.hardwareBackground && (
                <div className={styles.experienceItem}>
                  <span className={styles.experienceLabel}>
                    <Wrench size={16} />
                    Hardware
                  </span>
                  {getExperienceBadge(user.hardwareBackground)}
                </div>
              )}
              {user.pythonExperience && (
                <div className={styles.experienceItem}>
                  <span className={styles.experienceLabel}>
                    <Code size={16} />
                    Python
                  </span>
                  {getExperienceBadge(user.pythonExperience)}
                </div>
              )}
              {user.ros2Experience && (
                <div className={styles.experienceItem}>
                  <span className={styles.experienceLabel}>
                    <Box size={16} />
                    ROS 2
                  </span>
                  {getExperienceBadge(user.ros2Experience)}
                </div>
              )}
              {user.roboticsExperience && (
                <div className={styles.experienceItem}>
                  <span className={styles.experienceLabel}>
                    <Cpu size={16} />
                    Robotics
                  </span>
                  {getExperienceBadge(user.roboticsExperience)}
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );

  return createPortal(modalContent, document.body);
}
