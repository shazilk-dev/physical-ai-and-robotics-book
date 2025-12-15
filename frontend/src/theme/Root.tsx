/**
 * Root Theme Wrapper
 * Adds ChatWidget to all pages (client-side only)
 */

import React from "react";
import ChatWidget from "@site/src/components/ChatWidget/ChatWidget";

export default function Root({ children }) {
  return (
    <>
      {children}
      {/* ChatWidget handles its own SSR check */}
      <ChatWidget />
    </>
  );
}
