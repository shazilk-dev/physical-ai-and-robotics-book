import React, { useState, useEffect } from "react";
import { createPortal } from "react-dom";
import { signUp } from "@site/src/lib/auth-client";
import { CheckCircle2, XCircle, AlertCircle, Eye, EyeOff } from "lucide-react";
import styles from "./AuthModal.module.css";

interface SignupModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToLogin: () => void;
}

type ExperienceLevel = "none" | "beginner" | "intermediate" | "advanced";

interface PasswordStrength {
  score: number; // 0-4
  label: string;
  color: string;
  feedback: string[];
}

export default function SignupModal({
  isOpen,
  onClose,
  onSwitchToLogin,
}: SignupModalProps) {
  const [step, setStep] = useState(1);
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [mounted, setMounted] = useState(false);

  // Form touched states
  const [nameTouched, setNameTouched] = useState(false);
  const [emailTouched, setEmailTouched] = useState(false);
  const [passwordTouched, setPasswordTouched] = useState(false);
  const [confirmPasswordTouched, setConfirmPasswordTouched] = useState(false);
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  // Reset form when modal opens/closes
  useEffect(() => {
    if (!isOpen) {
      setStep(1);
      setName("");
      setEmail("");
      setPassword("");
      setConfirmPassword("");
      setError("");
      setNameTouched(false);
      setEmailTouched(false);
      setPasswordTouched(false);
      setConfirmPasswordTouched(false);
      setShowPassword(false);
      setShowConfirmPassword(false);
      setSoftwareBackground("beginner");
      setHardwareBackground("beginner");
      setPythonExperience("beginner");
      setRos2Experience("none");
      setRoboticsExperience("beginner");
    }
  }, [isOpen]);

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

  // Validation functions
  const isValidEmail = (email: string): boolean => {
    return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  };

  const calculatePasswordStrength = (pwd: string): PasswordStrength => {
    let score = 0;
    const feedback: string[] = [];

    if (pwd.length === 0) {
      return { score: 0, label: "", color: "", feedback: [] };
    }

    // Length check
    if (pwd.length >= 8) score++;
    else feedback.push("At least 8 characters");

    if (pwd.length >= 12) score++;

    // Character variety
    if (/[a-z]/.test(pwd) && /[A-Z]/.test(pwd)) {
      score++;
    } else {
      feedback.push("Mix of uppercase and lowercase");
    }

    if (/\d/.test(pwd)) {
      score++;
    } else {
      feedback.push("Include numbers");
    }

    if (/[^a-zA-Z\d]/.test(pwd)) {
      score++;
    } else {
      feedback.push("Include special characters");
    }

    // Determine strength label and color
    let label = "";
    let color = "";

    if (score === 0 || score === 1) {
      label = "Weak";
      color = "#ef4444";
    } else if (score === 2) {
      label = "Fair";
      color = "#f59e0b";
    } else if (score === 3) {
      label = "Good";
      color = "#10b981";
    } else {
      label = "Strong";
      color = "#059669";
    }

    return { score: Math.min(score, 4), label, color, feedback };
  };

  const passwordStrength = calculatePasswordStrength(password);

  const getNameValidation = () => {
    if (!nameTouched) return null;
    if (!name.trim()) return { valid: false, message: "Name is required" };
    if (name.trim().length < 2) return { valid: false, message: "Name must be at least 2 characters" };
    return { valid: true, message: "" };
  };

  const getEmailValidation = () => {
    if (!emailTouched) return null;
    if (!email) return { valid: false, message: "Email is required" };
    if (!isValidEmail(email)) return { valid: false, message: "Please enter a valid email" };
    return { valid: true, message: "" };
  };

  const getPasswordValidation = () => {
    if (!passwordTouched) return null;
    if (!password) return { valid: false, message: "Password is required" };
    if (password.length < 8) return { valid: false, message: "Password must be at least 8 characters" };
    if (passwordStrength.score < 2) return { valid: false, message: "Password is too weak" };
    return { valid: true, message: "" };
  };

  const getConfirmPasswordValidation = () => {
    if (!confirmPasswordTouched) return null;
    if (!confirmPassword) return { valid: false, message: "Please confirm your password" };
    if (password !== confirmPassword) return { valid: false, message: "Passwords do not match" };
    return { valid: true, message: "" };
  };

  const nameValidation = getNameValidation();
  const emailValidation = getEmailValidation();
  const passwordValidation = getPasswordValidation();
  const confirmPasswordValidation = getConfirmPasswordValidation();

  const isStep1Valid =
    name.trim().length >= 2 &&
    isValidEmail(email) &&
    password.length >= 8 &&
    passwordStrength.score >= 2 &&
    password === confirmPassword;

  const validateStep1 = () => {
    setNameTouched(true);
    setEmailTouched(true);
    setPasswordTouched(true);
    setConfirmPasswordTouched(true);

    if (!name.trim()) {
      setError("Name is required");
      return false;
    }
    if (!email.trim() || !isValidEmail(email)) {
      setError("Valid email is required");
      return false;
    }
    if (password.length < 8) {
      setError("Password must be at least 8 characters");
      return false;
    }
    if (passwordStrength.score < 2) {
      setError("Please choose a stronger password");
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
        // Provide user-friendly error messages
        const errorMessage = result.error.message || "Failed to create account";
        if (errorMessage.toLowerCase().includes("already exists") || errorMessage.toLowerCase().includes("duplicate")) {
          setError("An account with this email already exists. Please sign in instead.");
        } else if (errorMessage.toLowerCase().includes("network")) {
          setError("Network error. Please check your connection and try again.");
        } else if (errorMessage.toLowerCase().includes("invalid")) {
          setError("Invalid information provided. Please check your details and try again.");
        } else {
          setError(errorMessage);
        }
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
      setError("Unable to create account. Please try again later.");
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

        {error && (
          <div className={styles.error}>
            <AlertCircle size={16} />
            <span>{error}</span>
          </div>
        )}

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
              <div className={styles.inputWrapper}>
                <input
                  id="name"
                  type="text"
                  value={name}
                  onChange={(e) => {
                    setName(e.target.value);
                    if (!nameTouched) setNameTouched(true);
                  }}
                  onBlur={() => setNameTouched(true)}
                  placeholder="John Doe"
                  className={nameValidation ? (nameValidation.valid ? styles.inputValid : styles.inputInvalid) : ""}
                />
                {nameValidation && (
                  <span className={styles.validationIcon}>
                    {nameValidation.valid ? (
                      <CheckCircle2 size={18} className={styles.iconValid} />
                    ) : (
                      <XCircle size={18} className={styles.iconInvalid} />
                    )}
                  </span>
                )}
              </div>
              {nameValidation && !nameValidation.valid && (
                <span className={styles.validationMessage}>{nameValidation.message}</span>
              )}
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="signup-email">Email</label>
              <div className={styles.inputWrapper}>
                <input
                  id="signup-email"
                  type="email"
                  value={email}
                  onChange={(e) => {
                    setEmail(e.target.value);
                    if (!emailTouched) setEmailTouched(true);
                  }}
                  onBlur={() => setEmailTouched(true)}
                  placeholder="your@email.com"
                  autoComplete="email"
                  className={emailValidation ? (emailValidation.valid ? styles.inputValid : styles.inputInvalid) : ""}
                />
                {emailValidation && (
                  <span className={styles.validationIcon}>
                    {emailValidation.valid ? (
                      <CheckCircle2 size={18} className={styles.iconValid} />
                    ) : (
                      <XCircle size={18} className={styles.iconInvalid} />
                    )}
                  </span>
                )}
              </div>
              {emailValidation && !emailValidation.valid && (
                <span className={styles.validationMessage}>{emailValidation.message}</span>
              )}
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="signup-password">Password</label>
              <div className={styles.inputWrapper}>
                <input
                  id="signup-password"
                  type={showPassword ? "text" : "password"}
                  value={password}
                  onChange={(e) => {
                    setPassword(e.target.value);
                    if (!passwordTouched) setPasswordTouched(true);
                  }}
                  onBlur={() => setPasswordTouched(true)}
                  placeholder="••••••••"
                  autoComplete="new-password"
                  className={passwordValidation ? (passwordValidation.valid ? styles.inputValid : styles.inputInvalid) : ""}
                />
                <button
                  type="button"
                  onClick={() => setShowPassword(!showPassword)}
                  className={styles.passwordToggle}
                  aria-label={showPassword ? "Hide password" : "Show password"}
                >
                  {showPassword ? <EyeOff size={18} /> : <Eye size={18} />}
                </button>
                {passwordValidation && (
                  <span className={styles.validationIcon}>
                    {passwordValidation.valid ? (
                      <CheckCircle2 size={18} className={styles.iconValid} />
                    ) : (
                      <XCircle size={18} className={styles.iconInvalid} />
                    )}
                  </span>
                )}
              </div>
              {password && passwordTouched && (
                <div className={styles.passwordStrength}>
                  <div className={styles.strengthMeter}>
                    <div
                      className={styles.strengthBar}
                      style={{
                        width: `${(passwordStrength.score / 4) * 100}%`,
                        backgroundColor: passwordStrength.color,
                      }}
                    />
                  </div>
                  {passwordStrength.label && (
                    <span
                      className={styles.strengthLabel}
                      style={{ color: passwordStrength.color }}
                    >
                      {passwordStrength.label}
                    </span>
                  )}
                </div>
              )}
              {passwordValidation && !passwordValidation.valid && (
                <span className={styles.validationMessage}>{passwordValidation.message}</span>
              )}
              {password && passwordStrength.feedback.length > 0 && (
                <ul className={styles.passwordFeedback}>
                  {passwordStrength.feedback.map((tip, index) => (
                    <li key={index}>{tip}</li>
                  ))}
                </ul>
              )}
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="confirm-password">Confirm Password</label>
              <div className={styles.inputWrapper}>
                <input
                  id="confirm-password"
                  type={showConfirmPassword ? "text" : "password"}
                  value={confirmPassword}
                  onChange={(e) => {
                    setConfirmPassword(e.target.value);
                    if (!confirmPasswordTouched) setConfirmPasswordTouched(true);
                  }}
                  onBlur={() => setConfirmPasswordTouched(true)}
                  placeholder="••••••••"
                  autoComplete="new-password"
                  className={confirmPasswordValidation ? (confirmPasswordValidation.valid ? styles.inputValid : styles.inputInvalid) : ""}
                />
                <button
                  type="button"
                  onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                  className={styles.passwordToggle}
                  aria-label={showConfirmPassword ? "Hide password" : "Show password"}
                >
                  {showConfirmPassword ? <EyeOff size={18} /> : <Eye size={18} />}
                </button>
                {confirmPasswordValidation && (
                  <span className={styles.validationIcon}>
                    {confirmPasswordValidation.valid ? (
                      <CheckCircle2 size={18} className={styles.iconValid} />
                    ) : (
                      <XCircle size={18} className={styles.iconInvalid} />
                    )}
                  </span>
                )}
              </div>
              {confirmPasswordValidation && !confirmPasswordValidation.valid && (
                <span className={styles.validationMessage}>{confirmPasswordValidation.message}</span>
              )}
            </div>

            <button
              type="submit"
              className={styles.submitButton}
              disabled={!isStep1Valid}
            >
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
                disabled={isLoading}
              >
                Back
              </button>
              <button
                type="submit"
                className={styles.submitButton}
                disabled={isLoading}
              >
                {isLoading ? (
                  <>
                    <span className={styles.spinner}></span>
                    Creating Account...
                  </>
                ) : (
                  "Create Account"
                )}
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
