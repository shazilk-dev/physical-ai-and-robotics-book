import React, { useState, useEffect } from "react";
import LoginModal from "./LoginModal";
import SignupModal from "./SignupModal";
import ProfileModal from "./ProfileModal";
import styles from "./AuthButtons.module.css";

// Backend URL - detect environment safely in browser
const getBackendUrl = () => {
  // Check if we have a custom API URL set (from environment variables)
  if (typeof window !== 'undefined' && (window as any).docusaurus?.siteConfig?.customFields?.apiUrl) {
    const apiUrl = (window as any).docusaurus.siteConfig.customFields.apiUrl;
    // Remove /api/v1 suffix to get base URL
    return apiUrl.replace('/api/v1', '');
  }

  // Check if running on production domain (Vercel)
  if (typeof window !== 'undefined' &&
      (window.location.hostname.includes('vercel.app') ||
       window.location.hostname.includes('physical-ai-robotics-book'))) {
    return 'https://physical-ai-and-robotics-book.onrender.com';
  }

  // Default to localhost for development
  return 'http://localhost:8000';
};

const BACKEND_URL = getBackendUrl();

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

interface Session {
  user: User;
}

export default function AuthButtons() {
  const [showLogin, setShowLogin] = useState(false);
  const [showSignup, setShowSignup] = useState(false);
  const [showProfile, setShowProfile] = useState(false);
  const [session, setSession] = useState<Session | null>(null);
  const [loading, setLoading] = useState(true);
  const [showMenu, setShowMenu] = useState(false);

  useEffect(() => {
    checkSession();
  }, []);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      const target = event.target as HTMLElement;
      if (showMenu && !target.closest(`.${styles.userMenu}`)) {
        setShowMenu(false);
      }
    };

    if (showMenu) {
      document.addEventListener("mousedown", handleClickOutside);
      return () =>
        document.removeEventListener("mousedown", handleClickOutside);
    }
  }, [showMenu]);

  const checkSession = async () => {
    try {
      const response = await fetch(`${BACKEND_URL}/api/auth/session`, {
        credentials: "include",
        headers: {
          "Content-Type": "application/json",
        },
      });

      if (response.ok) {
        const data = await response.json();
        if (data.user) {
          setSession({ user: data.user });
        }
      }
    } catch (error) {
      console.error("Error checking session:", error);
    } finally {
      setLoading(false);
    }
  };

  const handleSignOut = async () => {
    try {
      await fetch(`${BACKEND_URL}/api/auth/sign-out`, {
        method: "POST",
        credentials: "include",
      });
      setSession(null);
      setShowMenu(false);
      window.location.reload();
    } catch (error) {
      console.error("Error signing out:", error);
    }
  };

  if (loading) {
    return (
      <div className={styles.loadingSkeleton}>
        <div className={styles.skeletonAvatar}></div>
      </div>
    );
  }

  if (session?.user) {
    return (
      <div className={styles.userMenu}>
        <button
          className={styles.userButton}
          onClick={() => setShowMenu(!showMenu)}
          aria-label="User menu"
        >
          <div className={styles.avatar}>
            {session.user.name?.[0]?.toUpperCase() || "U"}
          </div>
          <span className={styles.userName}>{session.user.name || "User"}</span>
          <svg
            className={styles.chevron}
            width="12"
            height="12"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2.5"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <polyline points="6 9 12 15 18 9" />
          </svg>
        </button>

        {showMenu && (
          <div className={styles.dropdown}>
            <div className={styles.dropdownHeader}>
              <div className={styles.userInfo}>
                <strong>{session.user.name}</strong>
                <small>{session.user.email}</small>
              </div>
            </div>
            <hr className={styles.dropdownDivider} />
            <button
              className={styles.dropdownItem}
              onClick={() => {
                setShowProfile(true);
                setShowMenu(false);
              }}
            >
              <svg
                width="16"
                height="16"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
                <circle cx="12" cy="7" r="4" />
              </svg>
              View Profile
            </button>
            <button className={styles.dropdownItem} onClick={handleSignOut}>
              <svg
                width="16"
                height="16"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4" />
                <polyline points="16 17 21 12 16 7" />
                <line x1="21" y1="12" x2="9" y2="12" />
              </svg>
              Sign Out
            </button>
          </div>
        )}

        <ProfileModal
          isOpen={showProfile}
          onClose={() => setShowProfile(false)}
          user={session.user}
        />
      </div>
    );
  }

  return (
    <>
      <button
        onClick={() => setShowLogin(true)}
        className={styles.signInButton}
      >
        Sign In
      </button>

      <LoginModal
        isOpen={showLogin}
        onClose={() => setShowLogin(false)}
        onSwitchToSignup={() => {
          setShowLogin(false);
          setShowSignup(true);
        }}
      />

      <SignupModal
        isOpen={showSignup}
        onClose={() => setShowSignup(false)}
        onSwitchToLogin={() => {
          setShowSignup(false);
          setShowLogin(true);
        }}
      />
    </>
  );
}
