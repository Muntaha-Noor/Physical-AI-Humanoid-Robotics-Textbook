import React from 'react';
import clsx from 'clsx';
import Translate from '@docusaurus/Translate';
import styles from './styles.module.css';
import Heading from '@theme/Heading';

const CapstoneProject: React.FC = () => {
  return (
    <section className={styles.capstoneSection}>
      <div className="container">
        <div className="row">
          <div className="col col--10 col--offset-1">
            <div className={styles.capstoneContent}>
              <div className={styles.capstoneIcon}>
                <svg
                  xmlns="http://www.w3.org/2000/svg"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  className={styles.capstoneSvg}
                >
                  <path d="M12 8V4H8" />
                  <rect width="16" height="12" x="4" y="8" rx="2" />
                  <path d="M2 14h20M8 14v4M16 14v4M4 20h16" />
                </svg>
                <div className={styles.glowAccent}></div>
              </div>

              <Heading as="h2" className={styles.capstoneTitle}>
                <Translate>Autonomous Voice-Controlled Humanoid Project</Translate>
              </Heading>

              <p className={styles.capstoneDescription}>
                <Translate>
                  Synthesize all acquired knowledge in an advanced capstone project focusing on autonomous humanoid robotics.
                  Develop an intelligent voice-controlled humanoid capable of environmental perception,
                  natural language processing, and complex physical action execution.
                  This comprehensive project synthesizes all four modules into a sophisticated
                  demonstration of Physical AI and humanoid robotics capabilities.
                </Translate>
              </p>

              <div className={styles.capstoneFeatures}>
                <div className={styles.featureItem}>
                  <div className={styles.featureIcon}>🤖</div>
                  <div className={styles.featureText}>
                    <h4><Translate>Humanoid Control</Translate></h4>
                    <p><Translate>Implement ROS 2 control systems for humanoid locomotion and manipulation</Translate></p>
                  </div>
                </div>

                <div className={styles.featureItem}>
                  <div className={styles.featureIcon}>👁️</div>
                  <div className={styles.featureText}>
                    <h4><Translate>Perception System</Translate></h4>
                    <p><Translate>Integrate NVIDIA Isaac perception for environment understanding</Translate></p>
                  </div>
                </div>

                <div className={styles.featureItem}>
                  <div className={styles.featureIcon}>💬</div>
                  <div className={styles.featureText}>
                    <h4><Translate>Natural Language</Translate></h4>
                    <p><Translate>Use VLA models for voice command interpretation and response</Translate></p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default CapstoneProject;