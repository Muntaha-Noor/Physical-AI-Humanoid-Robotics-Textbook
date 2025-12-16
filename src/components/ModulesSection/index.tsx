import React from 'react';
import clsx from 'clsx';
import ModuleCard from '../ModuleCard';
import { ModuleData } from '../../types'; // Adjust path as needed
import styles from './styles.module.css';
import Heading from '@theme/Heading';
import Translate from '@docusaurus/Translate';

const MODULES: ModuleData[] = [
  {
    title: <Translate>Module 1: The Robotic Nervous System (ROS 2)</Translate>,
    description: <Translate>Master the middleware essentials for robot control, ROS 2 communication primitives, Python-to-ROS bridges using rclpy, and URDF fundamentals for humanoid robots.</Translate>,
    link: '/docs/modules/01-ros2', // Updated to module index page
  },
  {
    title: <Translate>Module 2: The Digital Twin (Gazebo & Unity)</Translate>,
    description: <Translate>Explore advanced simulation for realistic environment and sensor modeling, focusing on physics simulation in Gazebo and high-fidelity rendering in Unity.</Translate>,
    link: '/docs/modules/02-simulation', // Updated to module index page
  },
  {
    title: <Translate>Module 3: The AI-Robot Brain (NVIDIA Isaac)</Translate>,
    description: <Translate>Dive into advanced perception, photorealistic simulation, synthetic data generation, VSLAM, navigation, and humanoid path-planning using NVIDIA Isaac tools.</Translate>,
    link: '/docs/modules/03-isaac-ai', // Updated to module index page
  },
  {
    title: <Translate>Module 4: Vision-Language-Action (VLA)</Translate>,
    description: <Translate>Integrate large language models (LLMs) with humanoid robotics for voice-driven autonomous actions, covering OpenAI Whisper, cognitive planning, and a capstone project.</Translate>,
    link: '/docs/modules/04-vla', // Updated to module index page
  },
];

const ModulesSection: React.FC = () => {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <Heading as="h2" className="text--center">
          <Translate>Our Modules</Translate>
        </Heading>
        <div className={clsx('row', styles.modulesGrid)}>
          {MODULES.map((module, idx) => (
            <div key={idx} className={styles.moduleCardWrapper}>
              <ModuleCard {...module} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default ModulesSection;
