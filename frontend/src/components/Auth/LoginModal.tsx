import React, { useState, useEffect } from "react";
import { createPortal } from "react-dom";
import { signIn } from "@site/src/lib/auth-client";
import { CheckCircle2, XCircle, AlertCircle } from "lucide-react";
import styles from "./AuthModal.module.css";

interface LoginModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToSignup: () => void;
}

export default function LoginModal({
  isOpen,
  onClose,
  onSwitchToSignup,
}: LoginModalProps) {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [mounted, setMounted] = useState(false);
  const [emailTouched, setEmailTouched] = useState(false);
  const [passwordTouched, setPasswordTouched] = useState(false);
  const [showPassword, setShowPassword] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  // Reset form when modal opens/closes
  useEffect(() => {
    if (!isOpen) {
      setEmail("");
      setPassword("");
      setError("");
      setEmailTouched(false);
      setPasswordTouched(false);
      setShowPassword(false);
    }
  }, [isOpen]);

  if (!isOpen || !mounted) return null;

  const isValidEmail = (email: string): boolean => {
    return /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
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
    return { valid: true, message: "" };
  };

  const emailValidation = getEmailValidation();
  const passwordValidation = getPasswordValidation();
  const isFormValid = email && password && isValidEmail(email) && password.length >= 8;

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    // Mark all fields as touched for validation
    setEmailTouched(true);
    setPasswordTouched(true);

    if (!isFormValid) {
      return;
    }

    setIsLoading(true);

    try {
      const result = await signIn.email({
        email,
        password,
      });

      if (result.error) {
        // Provide user-friendly error messages
        const errorMessage = result.error.message || "Invalid email or password";
        if (errorMessage.toLowerCase().includes("invalid")) {
          setError("Invalid email or password. Please check your credentials and try again.");
        } else if (errorMessage.toLowerCase().includes("network")) {
          setError("Network error. Please check your connection and try again.");
        } else {
          setError(errorMessage);
        }
      } else {
        // Success - close modal and reload to update session
        onClose();
        window.location.reload();
      }
    } catch (err) {
      setError("Unable to sign in. Please try again later.");
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
          <h2>Welcome Back</h2>
          <p>Sign in to continue your learning journey</p>
        </div>

        <form onSubmit={handleSubmit} className={styles.form}>
          {error && (
            <div className={styles.error}>
              <AlertCircle size={16} />
              <span>{error}</span>
            </div>
          )}

          <div className={styles.formGroup}>
            <label htmlFor="email">Email</label>
            <div className={styles.inputWrapper}>
              <input
                id="email"
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
            <label htmlFor="password">Password</label>
            <div className={styles.inputWrapper}>
              <input
                id="password"
                type={showPassword ? "text" : "password"}
                value={password}
                onChange={(e) => {
                  setPassword(e.target.value);
                  if (!passwordTouched) setPasswordTouched(true);
                }}
                onBlur={() => setPasswordTouched(true)}
                placeholder="••••••••"
                autoComplete="current-password"
                className={passwordValidation ? (passwordValidation.valid ? styles.inputValid : styles.inputInvalid) : ""}
              />
              <button
                type="button"
                onClick={() => setShowPassword(!showPassword)}
                className={styles.passwordToggle}
                aria-label={showPassword ? "Hide password" : "Show password"}
              >
                {showPassword ? (
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24" />
                    <line x1="1" y1="1" x2="23" y2="23" />
                  </svg>
                ) : (
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
                    <circle cx="12" cy="12" r="3" />
                  </svg>
                )}
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
            {passwordValidation && !passwordValidation.valid && (
              <span className={styles.validationMessage}>{passwordValidation.message}</span>
            )}
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading || !isFormValid}
          >
            {isLoading ? (
              <>
                <span className={styles.spinner}></span>
                Signing in...
              </>
            ) : (
              "Sign In"
            )}
          </button>
        </form>

        <div className={styles.footer}>
          <p>
            Don't have an account?{" "}
            <button
              onClick={onSwitchToSignup}
              className={styles.linkButton}
              type="button"
            >
              Sign up
            </button>
          </p>
        </div>
      </div>
    </div>
  );

  return createPortal(modalContent, document.body);
}
