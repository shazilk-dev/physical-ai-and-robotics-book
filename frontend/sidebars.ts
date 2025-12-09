import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main book sidebar - clear, linear structure
  bookSidebar: [
    {
      type: "doc",
      id: "intro",
      label: "Getting Started",
    },
    {
      type: "category",
      label: "Module 1: ROS 2 Fundamentals",
      collapsed: false,
      link: {
        type: "doc",
        id: "module-01-ros2/overview",
      },
      items: [
        {
          type: "category",
          label: "ROS 2 Core Concepts",
          collapsed: false,
          items: [
            "module-01-ros2/ros2-fundamentals/1.1.1-architecture",
            "module-01-ros2/ros2-fundamentals/1.1.2-rclpy-patterns",
            "module-01-ros2/ros2-fundamentals/1.1.3-parameters-launch",
            "module-01-ros2/ros2-fundamentals/1.1.4-qos-realtime",
          ],
        },
        {
          type: "category",
          label: "Robot Description",
          collapsed: true,
          items: [
            "module-01-ros2/urdf-robot-description/urdf-basics",
            "module-01-ros2/urdf-robot-description/sensors-urdf",
            "module-01-ros2/urdf-robot-description/validating-kinematics",
            "module-01-ros2/urdf-robot-description/package-testing",
          ],
        },
      ],
    },

    // Dedicated Labs Section
    {
      type: "category",
      label: "Hands-On Labs",
      collapsed: true,
      link: {
        type: "doc",
        id: "labs/overview",
      },
      items: ["labs/lab01-ros2-basics"],
    },
  ],

  // Labs sidebar - for standalone labs view (optional)
  labsSidebar: [
    {
      type: "category",
      label: "Laboratory Exercises",
      collapsed: false,
      items: ["labs/overview", "labs/lab01-ros2-basics"],
    },
  ],
};
export default sidebars;
