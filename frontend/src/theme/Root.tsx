/**
 * Root Theme Wrapper
 * Adds ChatWidget to all pages
 */

import React from "react";
import ChatWidget from "@site/src/components/ChatWidget/ChatWidget";
import ExecutionEnvironment from "@docusaurus/ExecutionEnvironment";

export default function Root({ children }) {
  return (
    <>
      {children}
      {/* Only render ChatWidget in browser */}
      {ExecutionEnvironment.canUseDOM && <ChatWidget />}
    </>
  );
}
