/**
 * Root Theme Wrapper
 * Adds ChatWidget to all pages
 */

import React from "react";
import ChatWidget from "@site/src/components/ChatWidget/ChatWidget";

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
