/**
 * Root Theme Wrapper
 * Adds MobileSidebar and ChatWidget to all pages
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
          const MobileSidebar = require("@site/src/components/MobileSidebar/MobileSidebar").default;
          return (
            <>
              <MobileSidebar />
              <ChatWidget />
            </>
          );
        }}
      </BrowserOnly>
    </>
  );
}
