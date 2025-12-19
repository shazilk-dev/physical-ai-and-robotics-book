/**
 * MobileSidebar Component
 * Always-visible collapsed sidebar tab for mobile devices
 * Opens Docusaurus sidebar directly (separate from hamburger menu)
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

  // Monitor sidebar state to remove data attribute when closed
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const observer = new MutationObserver(() => {
      const navbarSidebar = document.querySelector('.navbar-sidebar');
      const isOpen = navbarSidebar && navbarSidebar.classList.contains('navbar-sidebar--show');

      if (!isOpen) {
        // Sidebar closed, remove our marker
        document.body.removeAttribute('data-index-triggered');
      }
    });

    observer.observe(document.body, {
      attributes: true,
      attributeFilter: ['class'],
      subtree: true,
    });

    return () => observer.disconnect();
  }, []);

  // Open sidebar by triggering Docusaurus sidebar toggle
  const openSidebar = () => {
    // Add a data attribute to distinguish from hamburger click
    document.body.setAttribute('data-index-triggered', 'true');

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
