/**
 * Root Theme Wrapper
 * Adds ChatWidget to all pages
 */

import React from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";

export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly>
        {() => {
          const ChatWidget = require("@site/src/components/ChatWidget/ChatWidget").default;
          return <ChatWidget />;
        }}
      </BrowserOnly>
    </>
  );
}
