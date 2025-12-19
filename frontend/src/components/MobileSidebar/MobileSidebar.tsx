/**
 * MobileSidebar Component
 * Always-visible collapsed sidebar tab for mobile devices
 * Opens Docusaurus native sidebar with full chapter index
 */

import React, { useState, useEffect } from "react";
import { BookOpen } from "lucide-react";
import styles from "./MobileSidebar.module.css";

export default function MobileSidebar() {
  const [isMobile, setIsMobile] = useState(false);

  // Mobile detection (<=996px = Docusaurus mobile breakpoint)
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const checkMobile = () => {
      setIsMobile(window.innerWidth <= 996);
    };

    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, []);

  // Toggle Docusaurus native sidebar
  const openSidebar = () => {
    const sidebarButton = document.querySelector('.navbar__toggle');
    if (sidebarButton instanceof HTMLElement) {
      sidebarButton.click();
    }
  };

  // Only render on mobile
  if (!isMobile) return null;

  return (
    <button
      className={styles.sidebarTab}
      onClick={openSidebar}
      aria-label="Open table of contents"
      title="Table of Contents"
    >
      <BookOpen size={22} strokeWidth={2.5} />
      <span className={styles.tabLabel}>INDEX</span>
    </button>
  );
}
