import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function AboutPage() {
  return (
    <Layout
      title="About the Book"
      description="Learn about the Physical AI & Humanoid Robotics book and its purpose">
      <div className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container ">
          <div className={styles.heroContent}>
            <div className="row">
              <div className="col col--8">
                <div className={styles.bookInfo}>
                  <Heading as="h1" className={clsx("hero__title", styles.mainTitle)}>
                    About This Book
                  </Heading>
                  <p className={clsx("hero__subtitle", styles.subtitle)}>
                    Physical AI & Humanoid Robotics: A Comprehensive Guide<br/>
                    <span className={styles.subSubtitle}>Building Intelligent Machines for the Real World</span>
                  </p>
                </div>
              </div>
              <div className="col col--4">
                <img
                  src="/img/rb.png"
                  alt="Book Cover"
                  className={styles.bookCoverImage}
                />
              </div>
            </div>
          </div>
        </div>
      </div>

      <main>
        <section className={styles.aboutSection}>
          <div className="container padding-vert--lg ">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <div className={styles.aboutCard}>
                  <Heading as="h2" className={styles.sectionTitle}>
                    About the Book
                  </Heading>

                  <p className={styles.aboutText}>
                    This comprehensive guide explores the fascinating intersection of Artificial Intelligence and Robotics,
                    focusing on the emerging field of Physical AI and Humanoid Robotics. The book is designed to take
                    readers from foundational concepts to advanced implementations in humanoid robot development.
                  </p>

                  <Heading as="h3" className={styles.sectionSubTitle}>
                    Book Overview
                  </Heading>

                  <p className={styles.aboutText}>
                    Physical AI represents the convergence of artificial intelligence with physical systems, enabling
                    machines to interact intelligently with the real world. This book covers everything from ROS 2
                    fundamentals to advanced humanoid robotics, including sensor integration, digital twins, AI
                    reasoning systems, and multimodal interaction capabilities.
                  </p>

                  <Heading as="h3" className={styles.sectionSubTitle}>
                    Who Should Read This Book?
                  </Heading>

                  <ul className={styles.aboutList}>
                    <li><strong>Students:</strong> Computer Science, Robotics, AI, and Engineering students seeking to understand modern robotics</li>
                    <li><strong>Developers:</strong> Software engineers looking to transition into robotics and AI development</li>
                    <li><strong>Researchers:</strong> Academics and researchers exploring Physical AI and embodied intelligence</li>
                    <li><strong>Enthusiasts:</strong> Anyone curious about the future of AI-powered robotics and humanoid systems</li>
                  </ul>

                  <Heading as="h3" className={styles.sectionSubTitle}>
                    Learning Approach
                  </Heading>

                  <p className={styles.aboutText}>
                    This book follows a hands-on, practical approach where theoretical concepts are reinforced with
                    practical implementations. Each module builds upon previous knowledge, gradually developing the
                    skills needed to create sophisticated AI-powered robotic systems.
                  </p>

                  <div className={styles.guidelinesBox}>
                    <Heading as="h3" className={styles.sectionSubTitle}>
                      Guidelines for Readers
                    </Heading>

                    <ul className={styles.guidelinesList}>
                      <li><strong>Start from the beginning:</strong> Each module builds on previous concepts</li>
                      <li><strong>Practice actively:</strong> Implement the examples and experiments as you learn</li>
                      <li><strong>Experiment boldly:</strong> Modify code examples to deepen your understanding</li>
                      <li><strong>Join the community:</strong> Engage with other learners and share your projects</li>
                      <li><strong>Be patient:</strong> Robotics is complex - take time to understand each concept</li>
                    </ul>
                  </div>

                  <Heading as="h3" className={styles.sectionSubTitle}>
                    Technology Stack
                  </Heading>

                  <p className={styles.aboutText}>
                    The book leverages industry-standard tools and frameworks including ROS 2 for robotic communication,
                    Gazebo and Unity for simulation, NVIDIA Isaac for AI-powered robotics, and modern AI frameworks
                    for perception and reasoning capabilities.
                  </p>

                  <div className={styles.ctaButton}>
                    <a
                      className="button button--secondary button--lg"
                      href="/docs/docs/intro"
                    >
                      Start Reading - Introduction Module
                    </a>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}

export default AboutPage;