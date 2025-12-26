import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.bookInfo}>
            <Heading as="h1" className={clsx("hero__title", styles.mainTitle)}>
              PHYSICAL AI & HUMANOID ROBOTICS
            </Heading>
            <p className={clsx("hero__subtitle", styles.subtitle)}>
              Building Intelligent Machines for the Real World<br/>
              <span className={styles.subSubtitle}>AI, Robotics, ROS 2 & Embodied Intelligence</span>
            </p>
            <p className={styles.focusLine}>
              From Sensors to Humanoid Intelligence
            </p>
            <div className={styles.technologiesLine}>
              <span className={styles.techItem}>ROS 2</span>
              <span className={styles.techSeparator}>•</span>
              <span className={styles.techItem}>Sensors</span>
              <span className={styles.techSeparator}>•</span>
              <span className={styles.techItem}>Digital Twins</span>
              <span className={styles.techSeparator}>•</span>
              <span className={styles.techItem}>AI Agents</span>
              <span className={styles.techSeparator}>•</span>
              <span className={styles.techItem}>Humanoid Systems</span>
            </div>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/docs/intro">
                Explore the Book - 5min ⏱️
              </Link>
            </div>
          </div>

          <div className={styles.robotImage}>
            <div className={styles.robotImageContainer}>
              {/* FIXED PATH: /static word remove kiya gaya hai */}
              <img
  src={require('@site/static/img/robot.jpeg').default}
  alt="Robot"
  className={styles.robotImageElement}
/>
              {/* CSS se banay gaye effects image ke peeche ya upar set honge */}
              <div className={styles.humanoidRobot}>
                <div className={styles.physicalHalf}></div>
                <div className={styles.digitalHalf}></div>
                <div className={styles.sensor}></div>
                <div className={styles.lidar}></div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Bracing the Digital Brain and Physical Protein">
      <HomepageHeader />
      <main>
        <section className={styles.modulesSection}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Book Modules</h2>
            <div className="row">
              {/* Modules are the same as before */}
              {[1, 2, 3, 4].map((i) => {
                const moduleDescriptions = [
                  "ROS 2 fundamentals, URDF, sensors, and robot nervous system architecture.",
                  "Unity simulation, Gazebo environments, and digital twin technologies.",
                  "Cognitive architectures, planning algorithms, and AI reasoning systems.",
                  "VLA systems, multimodal perception, and human-robot interaction."
                ];

                return (
                <div key={i} className="col col--3">
                  <div className={clsx('card', styles.moduleCard)}>
                    <div className="card__header">
                      <h3>Module {i}: {["Nervous System", "Digital Twin", "AI Brain", "Vision-Language-Action"][i-1]}</h3>
                    </div>
                    <div className="card__body">
                      <p>{moduleDescriptions[i-1]}</p>
                    </div>
                    <div className="card__footer">
                      <Link className="button button--primary button--block" to="#">
                        Start Learning
                      </Link>
                    </div>
                  </div>
                </div>
              )})}
            </div>
          </div>
        </section>

        {/* Weekly Curriculum Breakdown */}
        <section className={styles.curriculumSection}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Weekly Curriculum Breakdown</h2>
            <div className={styles.curriculumContainer}>
              <div className={styles.curriculumTimeline}>
                <div className={styles.timelineItem}>
                  <div className={styles.timelineHeader}>
                    <h3>Weeks 1-2: Introduction to Physical AI</h3>
                  </div>
                  <ul className={styles.timelineList}>
                    <li>Physical AI Foundations: Understanding convergence of AI with physical systems</li>
                    <li>Embodied Intelligence: Intelligence from environment interaction</li>
                    <li>Humanoid Robotics Landscape: Current state-of-the-art overview</li>
                    <li>Sensor Systems: LIDAR, cameras, IMUs, force/torque sensors</li>
                  </ul>
                </div>

                <div className={styles.timelineItem}>
                  <div className={styles.timelineHeader}>
                    <h3>Weeks 3-5: ROS 2 Fundamentals</h3>
                  </div>
                  <ul className={styles.timelineList}>
                    <li>ROS 2 Architecture: Nodes, topics, services, and actions</li>
                    <li>Package Development: Building ROS 2 packages with Python</li>
                    <li>Launch Files: Parameter management and system configuration</li>
                    <li>Communication Patterns: Different paradigms in ROS 2</li>
                  </ul>
                </div>

                <div className={styles.timelineItem}>
                  <div className={styles.timelineHeader}>
                    <h3>Weeks 6-7: Robot Simulation with Gazebo</h3>
                  </div>
                  <ul className={styles.timelineList}>
                    <li>Gazebo Environment: Setting up simulation environments</li>
                    <li>URDF/SDF Formats: Robot description and simulation formats</li>
                    <li>Physics Simulation: Accurate physics modeling and sensor simulation</li>
                    <li>Unity Integration: Advanced visualization using Unity</li>
                  </ul>
                </div>

                <div className={styles.timelineItem}>
                  <div className={styles.timelineHeader}>
                    <h3>Weeks 8-10: NVIDIA Isaac Platform</h3>
                  </div>
                  <ul className={styles.timelineList}>
                    <li>Isaac SDK: NVIDIA Isaac development platform for AI robotics</li>
                    <li>Isaac Sim: High-fidelity simulation environment</li>
                    <li>Perception and Manipulation: AI-powered capabilities</li>
                    <li>Reinforcement Learning: Learning-based approaches for control</li>
                    <li>Sim-to-Real Transfer: Techniques for simulation-to-reality transfer</li>
                  </ul>
                </div>

                <div className={styles.timelineItem}>
                  <div className={styles.timelineHeader}>
                    <h3>Weeks 11-12: Humanoid Robot Development</h3>
                  </div>
                  <ul className={styles.timelineList}>
                    <li>Kinematics and Dynamics: Understanding robot movement and balance</li>
                    <li>Bipedal Locomotion: Walking and balance control</li>
                    <li>Manipulation and Grasping: Using humanoid hands for interaction</li>
                    <li>Human-Robot Interaction: Natural interaction design</li>
                  </ul>
                </div>

                <div className={styles.timelineItem}>
                  <div className={styles.timelineHeader}>
                    <h3>Week 13: Conversational Robotics</h3>
                  </div>
                  <ul className={styles.timelineList}>
                    <li>Conversational AI: Integrating GPT models for interaction</li>
                    <li>Speech Recognition: Processing spoken commands</li>
                    <li>Multi-Modal Interaction: Combining speech, gesture, and vision</li>
                  </ul>
                </div>
              </div>

              <div className={styles.curriculumInfo}>
                <div className={styles.infoCard}>
                  <h3>Course Structure</h3>
                  <p>This comprehensive 13-week curriculum takes you from foundational concepts to advanced humanoid robotics, covering everything from ROS 2 fundamentals to conversational AI integration.</p>
                </div>

                <div className={styles.infoCard}>
                  <h3>Learning Path</h3>
                  <p>Progress from understanding physical AI fundamentals to implementing sophisticated humanoid robot systems with AI-powered perception, planning, and interaction capabilities.</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Important Key Concepts Section */}
        <section className={styles.keyConceptsSection}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Key Concepts</h2>
            <div className={styles.keyConceptsContainer}>
              <div className={styles.keyConceptsGrid}>
                <div className={styles.conceptCard}>
                  <div className={styles.conceptIcon}>🧠</div>
                  <h3 className={styles.conceptTitle}>Physical AI Fundamentals</h3>
                  <ul className={styles.conceptList}>
                    <li>Physical AI: Convergence of AI with physical systems</li>
                    <li>Embodied Cognition: Intelligence from environment interaction</li>
                    <li>Sim-to-Real Transfer: Simulation before real-world deployment</li>
                    <li>Perception-Action Loops: Sensing, processing, and acting cycles</li>
                    <li>Safety-First Design: Prioritizing safety in AI integration</li>
                  </ul>
                </div>

                <div className={styles.conceptCard}>
                  <div className={styles.conceptIcon}>⚙️</div>
                  <h3 className={styles.conceptTitle}>Core Technologies</h3>
                  <ul className={styles.conceptList}>
                    <li>ROS 2: Middleware for robotic system communication</li>
                    <li>URDF: Robot model description format</li>
                    <li>Digital Twins: Virtual replicas of physical systems</li>
                    <li>Behavior Trees: Structured robotic behavior organization</li>
                    <li>SLAM: Simultaneous Localization and Mapping</li>
                  </ul>
                </div>
              </div>

              <div className={styles.keyConceptsGrid}>
                <div className={styles.conceptCard}>
                  <div className={styles.conceptIcon}>🤖</div>
                  <h3 className={styles.conceptTitle}>AI Integration</h3>
                  <ul className={styles.conceptList}>
                    <li>Vision-Language-Action: Unified perception and action</li>
                    <li>LLM Integration: Large language models for planning</li>
                    <li>Sensor Fusion: Combining multiple sensor data</li>
                    <li>Cognitive Architecture: Robot reasoning frameworks</li>
                    <li>Multi-Modal Planning: Spatial, temporal, and resource planning</li>
                  </ul>
                </div>

                <div className={styles.conceptCard}>
                  <div className={styles.conceptIcon}>🚀</div>
                  <h3 className={styles.conceptTitle}>Applications</h3>
                  <ul className={styles.conceptList}>
                    <li>Autonomous Systems: Self-driving vehicles and drones</li>
                    <li>Humanoid Robotics: Human-like assistance and collaboration</li>
                    <li>Industrial Automation: Smart manufacturing systems</li>
                    <li>Healthcare Robotics: Medical assistance devices</li>
                    <li>Service Robotics: Domestic and commercial robots</li>
                  </ul>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}