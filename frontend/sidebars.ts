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

    // Chapters Section
    {
      type: "category",
      label: "📚 Core Chapters",
      collapsed: false,
      items: [
        "chapters/chapter-01-foundations",
        "chapters/chapter-02-kinematics-actuation",
      ],
    },

    // Module 1: ROS 2 Fundamentals
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
          label: "Robot Description (URDF)",
          collapsed: true,
          items: [
            "module-01-ros2/urdf-robot-description/urdf-basics",
            "module-01-ros2/urdf-robot-description/sensors-urdf",
            "module-01-ros2/urdf-robot-description/validating-kinematics",
            "module-01-ros2/urdf-robot-description/package-testing",
          ],
        },
        {
          type: "category",
          label: "Sensors & Proprioception",
          collapsed: true,
          items: [
            "module-01-ros2/sensors-proprioception/1.3.1-imu-encoder-basics",
            "module-01-ros2/sensors-proprioception/1.3.2-realsense-integration",
            "module-01-ros2/sensors-proprioception/1.3.3-sensor-fusion",
          ],
        },
        "module-01-ros2/quiz",
      ],
    },

    // Module 2: Architecture & Hardware
    {
      type: "category",
      label: "Module 2: Architecture & Hardware",
      collapsed: false,
      link: {
        type: "doc",
        id: "module-02-architecture/overview",
      },
      items: [
        {
          type: "category",
          label: "Mechanical Design",
          collapsed: true,
          items: [
            "module-02-architecture/mechanical-design/2.1.1-link-optimization",
            "module-02-architecture/mechanical-design/2.1.2-center-of-mass",
            "module-02-architecture/mechanical-design/2.1.3-material-selection",
          ],
        },
        {
          type: "category",
          label: "Actuation Systems",
          collapsed: true,
          items: [
            "module-02-architecture/actuation-systems/2.2.1-servo-motors",
            "module-02-architecture/actuation-systems/2.2.2-harmonic-drives",
            "module-02-architecture/actuation-systems/2.2.3-series-elastic-actuators",
          ],
        },
        {
          type: "category",
          label: "Edge Computing",
          collapsed: true,
          items: [
            "module-02-architecture/edge-compute/2.3.1-jetson-architecture",
            "module-02-architecture/edge-compute/2.3.2-cuda-optimization",
            "module-02-architecture/edge-compute/2.3.3-tensorrt-deployment",
          ],
        },
        {
          type: "category",
          label: "Power Management",
          collapsed: true,
          items: [
            "module-02-architecture/power-management/2.4.1-battery-systems",
            "module-02-architecture/power-management/2.4.2-thermal-design",
            "module-02-architecture/power-management/2.4.3-power-distribution",
          ],
        },
        "module-02-architecture/quiz",
      ],
    },

    // Dedicated Labs Section
    {
      type: "category",
      label: "🔬 Hands-On Labs",
      collapsed: true,
      link: {
        type: "doc",
        id: "labs/overview",
      },
      items: [
        "labs/lab01-ros2-basics",
        "labs/lab02-urdf-humanoid",
        "labs/lab03-realsense-integration",
        "labs/lab04-motor-control",
      ],
    },
  ],

  // Labs sidebar - for standalone labs view (optional)
  labsSidebar: [
    {
      type: "category",
      label: "Laboratory Exercises",
      collapsed: false,
      items: [
        "labs/overview",
        "labs/lab01-ros2-basics",
        "labs/lab02-urdf-humanoid",
        "labs/lab03-realsense-integration",
        "labs/lab04-motor-control",
      ],
    },
  ],
};
export default sidebars;
