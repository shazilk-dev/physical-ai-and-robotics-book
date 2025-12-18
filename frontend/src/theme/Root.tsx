/**
 * Root Theme Wrapper
 * Adds NavigationFAB and ChatWidget to all pages
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
          const NavigationFAB = require("@site/src/components/NavigationFAB/NavigationFAB").default;
          return (
            <>
              <NavigationFAB />
              <ChatWidget />
            </>
          );
        }}
      </BrowserOnly>
    </>
  );
}
