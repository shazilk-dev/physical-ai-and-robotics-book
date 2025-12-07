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
  // Main book sidebar - module-based structure
  bookSidebar: [
    {
      type: "doc",
      id: "intro",
      label: "Introduction",
    },
    {
      type: "category",
      label: "Module 1: ROS 2 Nervous System",
      collapsed: false,
      items: [
        "module-01-ros2/overview",
        {
          type: "category",
          label: "1.1 ROS 2 Fundamentals",
          items: [
            "module-01-ros2/ros2-fundamentals/1.1.1-architecture",
            "module-01-ros2/ros2-fundamentals/1.1.2-rclpy-patterns",
            "module-01-ros2/ros2-fundamentals/1.1.3-parameters-launch",
            "module-01-ros2/ros2-fundamentals/1.1.4-qos-realtime",
          ],
        },
        {
          type: "category",
          label: "1.2 URDF & Robot Description",
          items: [
            "module-01-ros2/02-urdf-robot-description/1.2.1-urdf-basics",
            "module-01-ros2/02-urdf-robot-description/1.2.2-sensors-urdf",
            "module-01-ros2/02-urdf-robot-description/1.2.3-validating-kinematics",
            "module-01-ros2/02-urdf-robot-description/1.2.4-package-testing",
          ],
        },
      ],
    },
  ],

  // Labs sidebar - separate from main content
  labsSidebar: [
    {
      type: "category",
      label: "Labs",
      collapsed: false,
      items: [
        {
          type: "link",
          label: "Lab 1: ROS 2 Basics",
          href: "/labs/lab01-ros2-basics/README",
        },
      ],
    },
  ],
};
export default sidebars;
