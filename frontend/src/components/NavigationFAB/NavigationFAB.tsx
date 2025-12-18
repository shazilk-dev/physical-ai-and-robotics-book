/**
 * NavigationFAB Component
 * Mobile-only floating action button for sidebar navigation
 * Appears on devices <=996px, programmatically opens Docusaurus sidebar
 */

import React, { useState, useEffect } from "react";
import { BookOpen } from "lucide-react";
import styles from "./NavigationFAB.module.css";

export default function NavigationFAB() {
  const [isMobile, setIsMobile] = useState(false);
  const [isSidebarOpen, setIsSidebarOpen] = useState(false);

  // Detect mobile viewport (<=996px = Docusaurus mobile breakpoint)
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const checkMobile = () => {
      setIsMobile(window.innerWidth <= 996);
    };

    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, []);

  const toggleSidebar = () => {
    // Trigger Docusaurus sidebar toggle by clicking hamburger menu
    const sidebarButton = document.querySelector('.navbar__toggle');
    if (sidebarButton instanceof HTMLElement) {
      sidebarButton.click();
      setIsSidebarOpen(!isSidebarOpen);
    }
  };

  // Only render on mobile
  if (!isMobile) return null;

  return (
    <button
      className={styles.navigationFAB}
      onClick={toggleSidebar}
      aria-label="Open navigation sidebar"
      aria-expanded={isSidebarOpen}
    >
      <BookOpen size={24} strokeWidth={2.5} />
    </button>
  );
}
