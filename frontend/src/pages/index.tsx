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
              {[1, 2, 3, 4].map((i) => (
                <div key={i} className="col col--3">
                  <div className={clsx('card', styles.moduleCard)}>
                    <div className="card__header">
                      <h3>Module {i}: {["Nervous System", "Digital Twin", "AI Brain", "Vision-Language-Action"][i-1]}</h3>
                    </div>
                    <div className="card__body">
                      <p>Exploring the core concepts of advanced robotics and physical artificial intelligence.</p>
                    </div>
                    <div className="card__footer">
                      <Link className="button button--primary button--block" to="#">
                        Start Learning
                      </Link>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}