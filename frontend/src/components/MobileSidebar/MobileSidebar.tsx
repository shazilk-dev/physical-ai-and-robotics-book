/**
 * MobileSidebar Component
 * Always-visible collapsed sidebar tab for mobile devices
 * Opens custom drawer with book chapters/modules index
 */

import React, { useState, useEffect } from "react";
import { BookOpen, X } from "lucide-react";
import { useDocsSidebar } from "@docusaurus/theme-common/internal";
import DocSidebarItems from "@theme/DocSidebarItems";
import styles from "./MobileSidebar.module.css";

export default function MobileSidebar() {
  const [isMobile, setIsMobile] = useState(false);
  const [isDrawerOpen, setIsDrawerOpen] = useState(false);
  const sidebar = useDocsSidebar();

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

  // Prevent body scroll when drawer is open
  useEffect(() => {
    if (isDrawerOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }
    return () => {
      document.body.style.overflow = '';
    };
  }, [isDrawerOpen]);

  // Close drawer when clicking sidebar link
  const handleLinkClick = () => {
    setIsDrawerOpen(false);
  };

  // Only render on mobile and when sidebar exists
  if (!isMobile || !sidebar) return null;

  return (
    <>
      {/* INDEX Tab - Always visible */}
      <button
        className={styles.sidebarTab}
        onClick={() => setIsDrawerOpen(true)}
        aria-label="Open table of contents"
        title="Table of Contents"
      >
        <BookOpen size={22} strokeWidth={2.5} />
        <span className={styles.tabLabel}>INDEX</span>
      </button>

      {/* Custom Sidebar Drawer */}
      {isDrawerOpen && (
        <>
          {/* Backdrop */}
          <div
            className={styles.backdrop}
            onClick={() => setIsDrawerOpen(false)}
          />

          {/* Drawer */}
          <div className={styles.drawer}>
            <div className={styles.drawerHeader}>
              <h2 className={styles.drawerTitle}>
                <BookOpen size={20} strokeWidth={2.5} />
                Table of Contents
              </h2>
              <button
                className={styles.closeButton}
                onClick={() => setIsDrawerOpen(false)}
                aria-label="Close sidebar"
              >
                <X size={20} />
              </button>
            </div>

            <div className={styles.drawerContent} onClick={handleLinkClick}>
              {/* Render Docusaurus sidebar content */}
              <DocSidebarItems
                items={sidebar.items}
                activePath={sidebar.path}
                level={1}
              />
            </div>
          </div>
        </>
      )}
    </>
  );
}
