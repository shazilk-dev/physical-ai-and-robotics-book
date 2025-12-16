import React, { useState, useEffect } from "react";
import { createPortal } from "react-dom";
import { signUp } from "@site/src/lib/auth-client";
import styles from "./AuthModal.module.css";

interface SignupModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToLogin: () => void;
}

type ExperienceLevel = "none" | "beginner" | "intermediate" | "advanced";

export default function SignupModal({
  isOpen,
  onClose,
  onSwitchToLogin,
}: SignupModalProps) {
  const [step, setStep] = useState(1);
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  // Step 1: Credentials
  const [name, setName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [confirmPassword, setConfirmPassword] = useState("");

  // Step 2: Background Questions
  const [softwareBackground, setSoftwareBackground] =
    useState<ExperienceLevel>("beginner");
  const [hardwareBackground, setHardwareBackground] =
    useState<ExperienceLevel>("beginner");
  const [pythonExperience, setPythonExperience] =
    useState<ExperienceLevel>("beginner");
  const [ros2Experience, setRos2Experience] = useState<ExperienceLevel>("none");
  const [roboticsExperience, setRoboticsExperience] =
    useState<ExperienceLevel>("beginner");

  if (!isOpen || !mounted) return null;

  const validateStep1 = () => {
    if (!name.trim()) {
      setError("Name is required");
      return false;
    }
    if (!email.trim() || !/\S+@\S+\.\S+/.test(email)) {
      setError("Valid email is required");
      return false;
    }
    if (password.length < 8) {
      setError("Password must be at least 8 characters");
      return false;
    }
    if (password !== confirmPassword) {
      setError("Passwords do not match");
      return false;
    }
    return true;
  };

  const handleNext = () => {
    setError("");
    if (step === 1 && validateStep1()) {
      setStep(2);
    }
  };

  const handleBack = () => {
    setError("");
    setStep(1);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setIsLoading(true);

    try {
      // Create account with Better Auth
      const result = await signUp.email({
        email,
        password,
        name,
      });

      if (result.error) {
        setError(result.error.message || "Failed to create account");
        setIsLoading(false);
        return;
      }

      // Update profile with background questions
      // Use the same backend URL detection as auth-client
      const getBackendUrl = () => {
        if (typeof window !== 'undefined' && (window as any).docusaurus?.siteConfig?.customFields?.apiUrl) {
          const apiUrl = (window as any).docusaurus.siteConfig.customFields.apiUrl;
          return apiUrl.replace('/api/v1', '');
        }
        if (typeof window !== 'undefined' &&
            (window.location.hostname.includes('vercel.app') ||
             window.location.hostname.includes('physical-ai-robotics-book'))) {
          return 'https://physical-ai-and-robotics-book.onrender.com';
        }
        return 'http://localhost:8000';
      };

      const BACKEND_URL = getBackendUrl();
      const profileResponse = await fetch(
        `${BACKEND_URL}/api/auth/user/update-profile`,
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          credentials: "include", // Include cookies
          body: JSON.stringify({
            softwareBackground,
            hardwareBackground,
            pythonExperience,
            ros2Experience,
            roboticsExperience,
          }),
        }
      );

      if (!profileResponse.ok) {
        console.warn("Profile update failed, but account was created");
      }

      // Success - close modal and reload to update session
      onClose();
      window.location.reload();
    } catch (err) {
      setError("Something went wrong. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  const handleBackdropClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
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
          <h2>Create Your Account</h2>
          <p>Join thousands learning Physical AI and Robotics</p>
          <div className={styles.progressBar}>
            <div className={styles.progressStep} data-active={step >= 1}>
              1
            </div>
            <div className={styles.progressLine} data-active={step >= 2} />
            <div className={styles.progressStep} data-active={step >= 2}>
              2
            </div>
          </div>
        </div>

        {error && <div className={styles.error}>{error}</div>}

        {step === 1 && (
          <form
            onSubmit={(e) => {
              e.preventDefault();
              handleNext();
            }}
            className={styles.form}
          >
            <div className={styles.formGroup}>
              <label htmlFor="name">Full Name</label>
              <input
                id="name"
                type="text"
                value={name}
                onChange={(e) => setName(e.target.value)}
                placeholder="John Doe"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="signup-email">Email</label>
              <input
                id="signup-email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="your@email.com"
                required
                autoComplete="email"
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="signup-password">Password</label>
              <input
                id="signup-password"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="••••••••"
                required
                autoComplete="new-password"
                minLength={8}
              />
              <small>Minimum 8 characters</small>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="confirm-password">Confirm Password</label>
              <input
                id="confirm-password"
                type="password"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                placeholder="••••••••"
                required
                autoComplete="new-password"
                minLength={8}
              />
            </div>

            <button type="submit" className={styles.submitButton}>
              Next: Background Questions
            </button>
          </form>
        )}

        {step === 2 && (
          <form onSubmit={handleSubmit} className={styles.form}>
            <p className={styles.stepDescription}>
              Help us personalize your learning experience
            </p>

            <div className={styles.formGroup}>
              <label htmlFor="software">Software Background</label>
              <select
                id="software"
                value={softwareBackground}
                onChange={(e) =>
                  setSoftwareBackground(e.target.value as ExperienceLevel)
                }
              >
                <option value="none">No experience</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="hardware">Hardware Background</label>
              <select
                id="hardware"
                value={hardwareBackground}
                onChange={(e) =>
                  setHardwareBackground(e.target.value as ExperienceLevel)
                }
              >
                <option value="none">No experience</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="python">Python Experience</label>
              <select
                id="python"
                value={pythonExperience}
                onChange={(e) =>
                  setPythonExperience(e.target.value as ExperienceLevel)
                }
              >
                <option value="none">No experience</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="ros2">ROS 2 Experience</label>
              <select
                id="ros2"
                value={ros2Experience}
                onChange={(e) =>
                  setRos2Experience(e.target.value as ExperienceLevel)
                }
              >
                <option value="none">No experience</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="robotics">Robotics Experience</label>
              <select
                id="robotics"
                value={roboticsExperience}
                onChange={(e) =>
                  setRoboticsExperience(e.target.value as ExperienceLevel)
                }
              >
                <option value="none">No experience</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.buttonGroup}>
              <button
                type="button"
                onClick={handleBack}
                className={styles.backButton}
              >
                Back
              </button>
              <button
                type="submit"
                className={styles.submitButton}
                disabled={isLoading}
              >
                {isLoading ? "Creating Account..." : "Create Account"}
              </button>
            </div>
          </form>
        )}

        <div className={styles.footer}>
          <p>
            Already have an account?{" "}
            <button
              onClick={onSwitchToLogin}
              className={styles.linkButton}
              type="button"
            >
              Sign in
            </button>
          </p>
        </div>
      </div>
    </div>
  );

  return createPortal(modalContent, document.body);
}
