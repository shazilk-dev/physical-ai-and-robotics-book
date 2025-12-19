/**
 * MobileSidebar Component
 * Always-visible collapsed sidebar tab for mobile devices
 * Sticks to sidebar edge when open, transforms to close button
 */

import React, { useState, useEffect } from "react";
import { BookOpen, X } from "lucide-react";
import styles from "./MobileSidebar.module.css";

export default function MobileSidebar() {
  const [isMobile, setIsMobile] = useState(false);
  const [isSidebarOpen, setIsSidebarOpen] = useState(false);

  // Mobile detection - OPTIMIZED with debounce
  useEffect(() => {
    if (typeof window === 'undefined') return;

    let timeoutId: NodeJS.Timeout;

    const checkMobile = () => {
      clearTimeout(timeoutId);
      timeoutId = setTimeout(() => {
        setIsMobile(window.innerWidth <= 996);
      }, 100); // 100ms debounce
    };

    // Initial check without delay
    setIsMobile(window.innerWidth <= 996);

    // Use passive listener for better scroll performance
    window.addEventListener('resize', checkMobile, { passive: true });

    return () => {
      clearTimeout(timeoutId);
      window.removeEventListener('resize', checkMobile);
    };
  }, []);

  // Monitor sidebar state - OPTIMIZED
  useEffect(() => {
    if (typeof window === 'undefined') return;

    let timeoutId: NodeJS.Timeout;

    // Debounced check to avoid excessive updates
    const checkSidebarState = () => {
      clearTimeout(timeoutId);
      timeoutId = setTimeout(() => {
        const navbarSidebar = document.querySelector('.navbar-sidebar');
        const isOpen = navbarSidebar?.classList.contains('navbar-sidebar--show');

        setIsSidebarOpen(!!isOpen);

        if (!isOpen) {
          document.body.removeAttribute('data-index-triggered');
        }
      }, 50); // 50ms debounce
    };

    // Observe only the navbar-sidebar element, not entire body
    const navbarSidebar = document.querySelector('.navbar-sidebar');

    if (!navbarSidebar) {
      // If sidebar doesn't exist yet, observe body with targeted config
      const observer = new MutationObserver(() => {
        const sidebar = document.querySelector('.navbar-sidebar');
        if (sidebar) {
          observer.disconnect();
          // Re-run this effect once sidebar exists
          checkSidebarState();
        }
      });

      observer.observe(document.body, {
        childList: true,
        subtree: true,
      });

      return () => {
        clearTimeout(timeoutId);
        observer.disconnect();
      };
    }

    // Observe only the navbar-sidebar element for class changes
    const observer = new MutationObserver(checkSidebarState);

    observer.observe(navbarSidebar, {
      attributes: true,
      attributeFilter: ['class'], // Only watch class attribute
    });

    // Initial check
    checkSidebarState();

    return () => {
      clearTimeout(timeoutId);
      observer.disconnect();
    };
  }, []);

  // Toggle sidebar - OPTIMIZED (cached query)
  const toggleSidebar = () => {
    const sidebarButton = document.querySelector('.navbar__toggle');
    if (!(sidebarButton instanceof HTMLElement)) return;

    if (!isSidebarOpen) {
      // Open sidebar with INDEX marker
      document.body.setAttribute('data-index-triggered', 'true');
    }

    sidebarButton.click();
  };

  // Only render on mobile
  if (!isMobile) return null;

  return (
    <button
      className={`${styles.sidebarTab} ${isSidebarOpen ? styles.attached : ''}`}
      onClick={toggleSidebar}
      aria-label={isSidebarOpen ? "Close table of contents" : "Open table of contents"}
      title={isSidebarOpen ? "Close" : "Table of Contents"}
    >
      {isSidebarOpen ? (
        <X size={20} strokeWidth={2.5} />
      ) : (
        <>
          <BookOpen size={20} strokeWidth={2.5} />
          <span className={styles.tabLabel}>INDEX</span>
        </>
      )}
    </button>
  );
}
