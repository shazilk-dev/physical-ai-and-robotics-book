import React from "react";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
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
            <Link
              className={styles.primaryButton}
              to="/docs/intro"
            >
              Start Reading →
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
          <div className={styles.robotIcon}>
            <svg viewBox="0 0 200 200" xmlns="http://www.w3.org/2000/svg">
              {/* Robot Head */}
              <rect
                x="70"
                y="40"
                width="60"
                height="50"
                rx="8"
                fill="#4a90e2"
              />
              {/* Eyes */}
              <circle cx="85" cy="60" r="6" fill="#fff" />
              <circle cx="115" cy="60" r="6" fill="#fff" />
              {/* Antenna */}
              <line
                x1="100"
                y1="40"
                x2="100"
                y2="25"
                stroke="#4a90e2"
                strokeWidth="3"
              />
              <circle cx="100" cy="22" r="5" fill="#7b68ee" />
              {/* Body */}
              <rect
                x="65"
                y="90"
                width="70"
                height="60"
                rx="10"
                fill="#5a9fd8"
              />
              {/* Arms */}
              <rect
                x="40"
                y="100"
                width="25"
                height="40"
                rx="8"
                fill="#4a90e2"
              />
              <rect
                x="135"
                y="100"
                width="25"
                height="40"
                rx="8"
                fill="#4a90e2"
              />
              {/* Legs */}
              <rect
                x="75"
                y="150"
                width="20"
                height="35"
                rx="6"
                fill="#4a90e2"
              />
              <rect
                x="105"
                y="150"
                width="20"
                height="35"
                rx="6"
                fill="#4a90e2"
              />
            </svg>
          </div>
        </div>
      </div>
      <div className={styles.heroStats}>
        <div className={styles.stat}>
          <div className={styles.statNumber}>10</div>
          <div className={styles.statLabel}>Chapters</div>
        </div>
        <div className={styles.stat}>
          <div className={styles.statNumber}>2025</div>
          <div className={styles.statLabel}>Latest Updates</div>
        </div>
        <div className={styles.stat}>
          <div className={styles.statNumber}>100+</div>
          <div className={styles.statLabel}>Code Examples</div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ icon, title, description, link }) {
  return (
    <div className={styles.featureCard}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
      {link && (
        <Link className={styles.featureLink} to={link}>
          Learn more →
        </Link>
      )}
    </div>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: "🤖",
      title: "Physical AI Fundamentals",
      description:
        "Understand embodied intelligence, the three eras of robotics, and the 2025 market inflection point driving humanoid adoption.",
      link: "/docs/chapters/chapter-01-foundations",
    },
    {
      icon: "⚙️",
      title: "Mechanical Design",
      description:
        "Master kinematics, inverse kinematics, actuation technologies (electric, hydraulic, QDD), and structural engineering principles.",
      link: "/docs/chapters/chapter-02-kinematics-actuation",
    },
    {
      icon: "👁️",
      title: "Edge Compute & Perception",
      description:
        "Explore real-time sensor fusion, NVIDIA Jetson Thor architecture, and perception pipelines for autonomous operation.",
      link: "#",
    },
    {
      icon: "🧠",
      title: "Generative AI & Learning",
      description:
        "Deep dive into Vision-Language-Action models, reinforcement learning, imitation learning, and behavior models.",
      link: "#",
    },
    {
      icon: "🏭",
      title: "Real-World Deployment",
      description:
        "Production case studies from Tesla, Figure AI, Boston Dynamics, and analysis of commercial applications.",
      link: "#",
    },
    {
      icon: "🔮",
      title: "Future Directions",
      description:
        "Explore ethics, safety standards, economic impacts, and the path to human-level dexterity and intelligence.",
      link: "#",
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
      title: "Part I: Foundations",
      chapters: [
        "Chapter 1: Foundations of Physical AI",
        "Chapter 2: Kinematics & Actuation",
      ],
      color: "#4a90e2",
    },
    {
      title: "Part II: Intelligence Systems",
      chapters: [
        "Chapter 3: Edge Compute & Perception",
        "Chapter 4: Control Systems",
        "Chapter 5: Planning & Navigation",
        "Chapter 6: Generative AI & Learning",
      ],
      color: "#7b68ee",
    },
    {
      title: "Part III: Applications & Future",
      chapters: [
        "Chapter 7: Real-World Applications",
        "Chapter 8: Economics & Manufacturing",
        "Chapter 9: Ethics & Safety",
        "Chapter 10: Future of Physical AI",
      ],
      color: "#50c878",
    },
  ];

  return (
    <section className={styles.bookStructure}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Book Structure</h2>
        <p className={styles.sectionSubtitle}>
          10 comprehensive chapters organized into three progressive parts
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
      icon: "🎓",
      label: "Engineering Students",
      description: "Perfect for robotics and AI coursework",
    },
    {
      icon: "💻",
      label: "Software Developers",
      description: "Transition into robotics development",
    },
    {
      icon: "🔬",
      label: "Researchers",
      description: "Reference for physical AI systems",
    },
    {
      icon: "🏢",
      label: "Industry Professionals",
      description: "Building humanoid platforms",
    },
  ];

  return (
    <section className={styles.audience}>
      <div className={styles.container}>
        <h2 className={styles.sectionTitle}>Who This Book Is For</h2>
        <div className={styles.audienceGrid}>
          {audiences.map((audience, idx) => (
            <div key={idx} className={styles.audienceCard}>
              <div className={styles.audienceIcon}>{audience.icon}</div>
              <h4 className={styles.audienceLabel}>{audience.label}</h4>
              <p className={styles.audienceDescription}>
                {audience.description}
              </p>
            </div>
          ))}
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
            to="/docs/chapters/chapter-01-foundations"
          >
            Begin Chapter 1 →
          </Link>
          <Link className={styles.ctaSecondaryButton} to="/docs/intro">
            Browse All Chapters
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
