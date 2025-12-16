import React from 'react';
import clsx from 'clsx';
import Translate from '@docusaurus/Translate';
import styles from './styles.module.css';
import Heading from '@theme/Heading';

const modulesData = [
  {
    id: 1,
    title: <Translate>Module 1: The Robotic Nervous System (ROS 2)</Translate>,
    description: <Translate>Master the middleware essentials for robot control, communication primitives, and Python-to-ROS bridges.</Translate>,
    points: [
      <Translate key="m1p1">ROS 2 communication primitives</Translate>,
      <Translate key="m1p2">Python-to-ROS bridges using rclpy</Translate>,
      <Translate key="m1p3">URDF fundamentals for humanoid robots</Translate>,
      <Translate key="m1p4">Node management and launch systems</Translate>
    ]
  },
  {
    id: 2,
    title: <Translate>Module 2: The Digital Twin (Gazebo & Unity)</Translate>,
    description: <Translate>Explore advanced simulation for realistic environment and sensor modeling.</Translate>,
    points: [
      <Translate key="m2p1">Physics simulation in Gazebo</Translate>,
      <Translate key="m2p2">High-fidelity rendering in Unity</Translate>,
      <Translate key="m2p3">Environment modeling and sensors</Translate>,
      <Translate key="m2p4">Digital twin creation techniques</Translate>
    ]
  },
  {
    id: 3,
    title: <Translate>Module 3: The AI-Robot Brain (NVIDIA Isaac)</Translate>,
    description: <Translate>Dive into high-performance perception, VSLAM, and navigation.</Translate>,
    points: [
      <Translate key="m3p1">Advanced perception systems</Translate>,
      <Translate key="m3p2">Visual Simultaneous Localization and Mapping</Translate>,
      <Translate key="m3p3">Navigation and path-planning</Translate>,
      <Translate key="m3p4">Humanoid locomotion algorithms</Translate>
    ]
  },
  {
    id: 4,
    title: <Translate>Module 4: Vision-Language-Action (VLA)</Translate>,
    description: <Translate>Integrate LLMs for natural language understanding and autonomous humanoid action.</Translate>,
    points: [
      <Translate key="m4p1">OpenAI Whisper integration</Translate>,
      <Translate key="m4p2">Cognitive planning systems</Translate>,
      <Translate key="m4p3">Vision-Language-Action models</Translate>,
      <Translate key="m4p4">Capstone project implementation</Translate>
    ]
  }
];

const WhatYoullLearn: React.FC = () => {
  return (
    <section className={styles.whatYoullLearnSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles.sectionTitle}>
              <Translate>What You'll Learn</Translate>
            </Heading>
            <p className={styles.sectionSubtitle}>
              <Translate>Our curriculum is structured into four comprehensive modules, each building upon the last to provide a holistic understanding of Physical AI and Humanoid Robotics:</Translate>
            </p>

            <div className={styles.modulesGrid}>
              {modulesData.map((module) => (
                <div key={module.id} className={styles.moduleCard}>
                  <div className={styles.moduleHeader}>
                    <h3 className={styles.moduleTitle}>{module.title}</h3>
                    <p className={styles.moduleDescription}>{module.description}</p>
                  </div>

                  <ul className={styles.modulePoints}>
                    {module.points.map((point, index) => (
                      <li key={index} className={styles.modulePoint}>
                        <span className={styles.bullet}>•</span> {point}
                      </li>
                    ))}
                  </ul>
                </div>
              ))}
            </div>

            {/* Learning Features Section */}
            <div className={styles.learningFeaturesSection}>
              <h3 className={styles.sectionSubtitle2}>
                <Translate>Learning Features</Translate>
              </h3>
              <div className={styles.featuresGrid}>
                <div className={styles.featureCard}>
                  <div className={styles.featureHeader}>
                    <h4 className={styles.featureTitle}>
                      <Translate>Detailed Explanations</Translate>
                    </h4>
                    <p className={styles.featureDescription}>
                      <Translate>Concepts broken down for advanced learners with comprehensive coverage of each topic.</Translate>
                    </p>
                  </div>
                </div>

                <div className={styles.featureCard}>
                  <div className={styles.featureHeader}>
                    <h4 className={styles.featureTitle}>
                      <Translate>Hands-on Code Examples</Translate>
                    </h4>
                    <p className={styles.featureDescription}>
                      <Translate>All code samples are tested and runnable in their respective simulation environments.</Translate>
                    </p>
                  </div>
                </div>

                <div className={styles.featureCard}>
                  <div className={styles.featureHeader}>
                    <h4 className={styles.featureTitle}>
                      <Translate>Clear Diagrams & Visuals</Translate>
                    </h4>
                    <p className={styles.featureDescription}>
                      <Translate>To aid understanding of complex systems with visual representations.</Translate>
                    </p>
                  </div>
                </div>

                <div className={styles.featureCard}>
                  <div className={styles.featureHeader}>
                    <h4 className={styles.featureTitle}>
                      <Translate>Capstone Project</Translate>
                    </h4>
                    <p className={styles.featureDescription}>
                      <Translate>A culmination of knowledge, allowing you to build an autonomous voice-controlled humanoid.</Translate>
                    </p>
                  </div>
                </div>
              </div>
            </div>

            {/* Capstone Project Section */}
            <div className={styles.capstoneCard}>
              <div className={styles.moduleHeader}>
                <h3 className={styles.moduleTitle}>
                  <Translate>Capstone Project</Translate>
                </h3>
                <p className={styles.moduleDescription}>
                  <Translate>A hands-on project where you will integrate all learned concepts to build an autonomous humanoid robot.</Translate>
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default WhatYoullLearn;