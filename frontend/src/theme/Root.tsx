/**
 * Root Theme Wrapper
 * Adds ChatWidget to all pages
 */

import React from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";
import ChatWidget from "@site/src/components/ChatWidget/ChatWidget";

export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly>{() => <ChatWidget />}</BrowserOnly>
    </>
  );
}
