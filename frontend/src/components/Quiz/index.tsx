import React from "react";
import BrowserOnly from "@docusaurus/BrowserOnly";
import QuizAccordion from "./QuizAccordion";

// Wrapper to ensure QuizAccordion only renders on client-side
export default function QuizAccordionWrapper(props) {
  return (
    <BrowserOnly fallback={<div>Loading quiz...</div>}>
      {() => <QuizAccordion {...props} />}
    </BrowserOnly>
  );
}
