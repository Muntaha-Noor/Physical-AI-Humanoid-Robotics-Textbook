import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { ModuleData } from '../../types'; // Adjust path as needed
import styles from './styles.module.css';
import Heading from '@theme/Heading';
import Translate from '@docusaurus/Translate';


const ModuleCard: React.FC<ModuleData> = ({ title, description, link }) => {
  return (
    <div className={clsx('card', styles.moduleCard)}>
      <div className="card__header">
        <Heading as="h3" className={styles.moduleCardTitle}>
          {title}
        </Heading>
      </div>
      <div className="card__body">
        {description}
      </div>
      <div className="card__footer">
        <Link
          className={clsx('button button--primary button--block', styles.gradientButton, styles.moduleCardButton)}
          to={link}>
          <Translate>Open Module →</Translate>
        </Link>
      </div>
    </div>
  );
};

export default ModuleCard;
