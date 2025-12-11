/**
 * Root Theme Wrapper
 * Adds ChatWidget to all pages and ensures sidebar collapse works
 */

import React, { useEffect } from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";
import ChatWidget from "@site/src/components/ChatWidget/ChatWidget";

export default function Root({ children }) {
  useEffect(() => {
    // Ensure sidebar collapse functionality works
    const fixSidebarInteraction = () => {
      const sidebarLinks = document.querySelectorAll(".menu__link--sublist");
      sidebarLinks.forEach((link) => {
        if (link instanceof HTMLElement) {
          // Ensure the link is clickable
          link.style.cursor = "pointer";
          link.style.pointerEvents = "auto";

          // Find and fix the caret too
          const caret = link.querySelector(".menu__caret");
          if (caret instanceof HTMLElement) {
            caret.style.cursor = "pointer";
            caret.style.pointerEvents = "auto";
          }
        }
      });
    };

    // Run immediately
    fixSidebarInteraction();

    // Run after a short delay to catch dynamically loaded content
    setTimeout(fixSidebarInteraction, 100);
    setTimeout(fixSidebarInteraction, 500);

    // Run on route changes
    const observer = new MutationObserver(fixSidebarInteraction);
    observer.observe(document.body, { childList: true, subtree: true });

    return () => observer.disconnect();
  }, []);

  return (
    <>
      {children}
      <BrowserOnly fallback={<div>Loading...</div>}>
        {() => <ChatWidget />}
      </BrowserOnly>
    </>
  );
}
