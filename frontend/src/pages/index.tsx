import React from "react";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import {
  Bot,
  Book,
  Cpu,
  Eye,
  Brain,
  Factory,
  Sparkles,
  GraduationCap,
  Code,
  FlaskConical,
  Building2,
  ArrowRight,
  Check,
} from "lucide-react";
import styles from "./index.module.css";

function HeroSection() {
  return (
    <header className={styles.hero}>
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <h1 className={styles.heroTitle}>
            Physical AI &{" "}
            <span className={styles.highlight}>Humanoid Robotics</span>
          </h1>
          <p className={styles.heroSubtitle}>
            Master the convergence of artificial intelligence and physical
            robotics. From foundational concepts to cutting-edge 2025
            developments.
          </p>
          <div className={styles.heroButtons}>
            <Link className={styles.primaryButton} to="/docs/intro">
              Start Reading <ArrowRight className={styles.buttonIcon} />
            </Link>
            <Link
              className={styles.secondaryButton}
              to="https://github.com/shazilk-dev/physical-ai-and-robotics-book"
            >
              View on GitHub
            </Link>
          </div>
        </div>
        <div className={styles.heroVisual}>
          <div className={styles.iconGrid}>
            <Bot className={styles.floatingIcon} size={64} strokeWidth={1.5} />
            <Cpu className={styles.floatingIcon} size={48} strokeWidth={1.5} />
            <Brain
              className={styles.floatingIcon}
              size={56}
              strokeWidth={1.5}
            />
          </div>
        </div>
      </div>
      <div className={styles.heroStats}>
        <div className={styles.stat}>
          <div className={styles.statNumber}>4</div>
          <div className={styles.statLabel}>Modules</div>
        </div>
        <div className={styles.stat}>
          <div className={styles.statNumber}>6+</div>
          <div className={styles.statLabel}>Labs</div>
        </div>
        <div className={styles.stat}>
          <div className={styles.statNumber}>2025</div>
          <div className={styles.statLabel}>Industry Focus</div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ icon: Icon, title, description, link }) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureIconWrapper}>
        <Icon className={styles.featureIcon} size={32} strokeWidth={1.5} />
      </div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
      {link && (
        <Link className={styles.featureLink} to={link}>
          Learn more <ArrowRight className={styles.linkIcon} size={16} />
        </Link>
      )}
    </div>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: Bot,
      title: "ROS 2 Fundamentals",
      description:
        "Learn the robotic nervous system: ROS 2 architecture, nodes, topics, services, and real-time communication patterns.",
      link: "/docs/module-01-ros2/overview",
    },
    {
      icon: Cpu,
      title: "URDF & Robot Description",
      description:
        "Master robot modeling with URDF/Xacro, sensor integration, kinematic validation, and robot description best practices.",
      link: "/docs/module-01-ros2/urdf-robot-description/urdf-basics",
    },
    {
      icon: Eye,
      title: "Sensors & Perception",
      description:
        "Learn sensor fusion, IMU calibration, RealSense depth cameras, and deploying perception stacks to NVIDIA Jetson edge devices.",
      link: "/docs/module-01-ros2/ros2-fundamentals/1.1.1-architecture",
    },
    {
      icon: Brain,
      title: "Hands-On Labs",
      description:
        "Build real ROS 2 nodes, create URDF robot descriptions, and deploy to hardware with step-by-step guided exercises.",
      link: "/docs/labs/overview",
    },
    {
      icon: Factory,
      title: "Real-Time Control",
      description:
        "Master QoS policies, latency budgeting, real-time constraints, and performance optimization for robot control loops.",
      link: "/docs/module-01-ros2/ros2-fundamentals/1.1.4-qos-realtime",
    },
    {
      icon: Sparkles,
      title: "2025 Industry Context",
      description:
        "Understand the current state of physical AI, humanoid robotics market, and cutting-edge technologies from NVIDIA, Unitree, and more.",
      link: "/docs/intro",
    },
  ];

  return (
    <section className={styles.features}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>What You'll Learn</h2>
        <p className={styles.sectionSubtitle}>
          Comprehensive coverage from mechanical foundations to AI-powered
          behavior models
        </p>
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <FeatureCard key={idx} {...feature} />
          ))}
        </div>
      </div>
    </section>
  );
}

function BookStructure() {
  const parts = [
    {
      title: "Module 1: ROS 2 Nervous System",
      chapters: [
        "1.1.1 ROS 2 Architecture",
        "1.1.2 rclpy Patterns",
        "1.1.3 Parameters & Launch Files",
        "1.1.4 QoS & Real-Time",
        "1.2.1 URDF Basics",
        "1.2.2 Sensors in URDF",
      ],
      color: "#3B82F6",
    },
    {
      title: "Module 2: Digital Twin (Coming Soon)",
      chapters: [
        "Gazebo Simulation",
        "Physics & Contacts",
        "Sensor Simulation",
        "Unity Integration",
      ],
      color: "#8B5CF6",
    },
    {
      title: "Module 3: AI-Robot Brain (Coming Soon)",
      chapters: [
        "NVIDIA Isaac Sim",
        "Isaac ROS Perception",
        "Nav2 Navigation",
        "Edge Deployment",
      ],
      color: "#10B981",
    },
    {
      title: "Module 4: VLA Models (Coming Soon)",
      chapters: [
        "Vision-Language-Action",
        "Voice Commands",
        "LLM Integration",
        "Capstone Project",
      ],
      color: "#F59E0B",
    },
  ];

  return (
    <section className={styles.bookStructure}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Course Modules</h2>
        <p className={styles.sectionSubtitle}>
          4 comprehensive modules covering ROS 2, simulation, AI integration,
          and voice-controlled autonomy
        </p>
        <div className={styles.partsGrid}>
          {parts.map((part, idx) => (
            <div
              key={idx}
              className={styles.partCard}
              style={{ borderTopColor: part.color }}
            >
              <h3 className={styles.partTitle}>{part.title}</h3>
              <ul className={styles.chapterList}>
                {part.chapters.map((chapter, chIdx) => (
                  <li key={chIdx} className={styles.chapterItem}>
                    <Check className={styles.checkIcon} size={16} />
                    {chapter}
                  </li>
                ))}
              </ul>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function AudienceSection() {
  const audiences = [
    {
      icon: GraduationCap,
      label: "Engineering Students",
      description: "Perfect for robotics and AI coursework",
    },
    {
      icon: Code,
      label: "Software Developers",
      description: "Transition into robotics development",
    },
    {
      icon: FlaskConical,
      label: "Researchers",
      description: "Reference for physical AI systems",
    },
    {
      icon: Building2,
      label: "Industry Professionals",
      description: "Building humanoid platforms",
    },
  ];

  return (
    <section className={styles.audience}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Who This Book Is For</h2>
        <div className={styles.audienceGrid}>
          {audiences.map((audience, idx) => {
            const Icon = audience.icon;
            return (
              <div key={idx} className={styles.audienceCard}>
                <Icon
                  className={styles.audienceIcon}
                  size={40}
                  strokeWidth={1.5}
                />
                <h4 className={styles.audienceLabel}>{audience.label}</h4>
                <p className={styles.audienceDescription}>
                  {audience.description}
                </p>
              </div>
            );
          })}
        </div>
        <div className={styles.prerequisites}>
          <p>
            <strong>Prerequisites:</strong> Basic Python knowledge and
            undergraduate-level mathematics (linear algebra, calculus). No prior
            robotics experience required.
          </p>
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.cta}>
      <div className={styles.container}>
        <h2 className={styles.ctaTitle}>Ready to Master Physical AI?</h2>
        <p className={styles.ctaSubtitle}>
          Start your journey into the future of embodied intelligence
        </p>
        <div className={styles.ctaButtons}>
          <Link
            className={styles.ctaPrimaryButton}
            to="/docs/module-01-ros2/overview"
          >
            Begin Module 1 <ArrowRight className={styles.buttonIcon} />
          </Link>
          <Link className={styles.ctaSecondaryButton} to="/docs/intro">
            View Full Book
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Comprehensive guide to Physical AI and Humanoid Robotics - from foundational concepts to cutting-edge 2025 developments"
    >
      <HeroSection />
      <FeaturesSection />
      <BookStructure />
      <AudienceSection />
      <CTASection />
    </Layout>
  );
}
