import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Translate from '@docusaurus/Translate';
import styles from './styles.module.css';

const HeroSection: React.FC = () => {
  return (
    <header className={clsx(styles.heroBanner)}>
      <div className={styles.heroContainer}>
        <div className={styles.contentAndImage}>
          <div className={styles.textContent}>
            <h1 className={styles.heroTitle}>
              <Translate>Physical AI & Humanoid Robotics Textbook</Translate>
            </h1>
            <p className={styles.heroSubtitle}>
              <Translate>
                A comprehensive guide to cutting-edge Physical AI and humanoid robotics with interactive RAG chatbot.
              </Translate>
            </p>
            <div className={styles.buttons}>
              <Link
                className={clsx(styles.heroButton)}
                to="/docs/intro">
                <Translate>Explore the Textbook</Translate>
              </Link>
            </div>
          </div>
          <div className={styles.robotContainer}>
            <div className={styles.robot3DContainer}>
              {/* 3D Robot Model would be loaded here */}
              <img
                src={require('../../robort.png').default}
                alt="Humanoid Robot"
                className={styles.robotImage}
              />
              {/* In a real implementation, a 3D model would replace this image */}
            </div>
          </div>
        </div>
      </div>
    </header>
  );
};

export default HeroSection;
