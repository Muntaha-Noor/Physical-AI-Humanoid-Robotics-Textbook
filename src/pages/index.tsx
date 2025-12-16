import type {ReactNode} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import Translate from '@docusaurus/Translate';

import HeroSection from '../components/HeroSection'; // Import the new HeroSection component
import ModulesSection from '../components/ModulesSection'; // Import the ModulesSection component
import WhatYoullLearn from '../components/WhatYoullLearn'; // Import the WhatYoullLearn component
import CapstoneProject from '../components/CapstoneProject'; // Import the CapstoneProject component
import styles from './index.module.css'; // Keep existing styles if needed, or update

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`} // Simplified title
      description="A comprehensive guide to cutting-edge Physical AI and humanoid robotics.">
      <HeroSection /> {/* Use the custom HeroSection */}
      <main>
        <section className={styles.introSection}>
          <div className="container">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <Heading as="h2" className="text--center">
                  <Translate>Unlock the Future of Humanoid Robotics</Translate>
                </Heading>
                <p className="text--center">
                  <Translate>
                    Dive deep into the fascinating world where artificial intelligence meets physical embodiment. 
                    This textbook is meticulously crafted for advanced Computer Science and Robotics students, 
                    offering a unique blend of theoretical knowledge and practical application. 
                    From the foundational principles of ROS 2 to the intricate dance of Vision-Language-Action systems, 
                    prepare to build, simulate, and program the next generation of intelligent machines.
                  </Translate>
                </p>
              </div>
            </div>
          </div>
        </section>

        {/* Glow-accent section divider */}
        <div className={styles.glowDivider}></div>

        <section className={styles.curriculumSection}>
          <div className="container">
            <div className="row">
              <div className="col col--10 col--offset-1">
                <Heading as="h2" className="text--center">
                  <Translate>Academic Foundation & Research Approach</Translate>
                </Heading>
                <div className="row">
                  <div className="col col--6">
                    <p>
                      <Translate>
                        This textbook is grounded in rigorous academic principles, bridging theoretical foundations with practical implementation in humanoid robotics. Each module follows a simulation-first approach that emphasizes reproducibility and research-driven learning methodologies.
                      </Translate>
                    </p>
                  </div>
                  <div className="col col--6">
                    <p>
                      <Translate>
                        Our curriculum philosophy centers on Physical AI—the intersection of artificial intelligence and embodied systems. Students will explore how theoretical concepts translate into real-world humanoid capabilities through systematic, research-oriented methodologies.
                      </Translate>
                    </p>
                  </div>
                </div>
                <div className="row" style={{marginTop: '2rem'}}>
                  <div className="col col--4">
                    <p>
                      <strong><Translate>Simulation-Driven Learning</Translate></strong><Translate>: Master concepts through Gazebo and Unity environments before real-world implementation.</Translate>
                    </p>
                  </div>
                  <div className="col col--4">
                    <p>
                      <strong><Translate>Research-Oriented Curriculum</Translate></strong><Translate>: Develop skills in experimental design, data collection, and analysis for robotics research.</Translate>
                    </p>
                  </div>
                  <div className="col col--4">
                    <p>
                      <strong><Translate>Theory-to-Practice Bridge</Translate></strong><Translate>: Translate advanced AI concepts into functional humanoid behaviors and control systems.</Translate>
                    </p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>


        <ModulesSection /> {/* Integrate ModulesSection here */}

        {/* Add new sections based on textbook content */}
        <WhatYoullLearn />
        <CapstoneProject />
      </main>
    </Layout>
  );
}
