import type {Config} from '@docusaurus/types';
import {themes as prismThemes} from 'prism-react-renderer';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A complete Docusaurus-based textbook with an integrated RAG chatbot.',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  // url: 'https://spect-kit-plus.github.io',
  url: 'https://Muntaha-Noor.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  // baseUrl: '/Physical-AI-Humanoid-Robotics-Textbook/', // Adjusted for GitHub Pages deployment
   baseUrl: '/Physical-AI-Humanoid-Robotics-Textbook/',
  // GitHub pages deployment config.
  // organizationName: 'spect-kit-plus', // Your GitHub org/user name.
  organizationName: 'Muntaha-Noor',
  // projectName: 'Hakathon_01', // Your repo name.
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook', 

  onBrokenLinks: 'ignore',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // Added Urdu locale
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/spect-kit-plus/Hakathon_01/tree/main/Humanoid_Robotics/',
        },
        blog: false, // Removed blog functionality as not required
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg', // Placeholder, will be replaced with custom image
    colorMode: {
      defaultMode: 'dark', // Default to dark mode for futuristic theme
      disableSwitch: false, // Allow users to switch theme
      respectPrefersColorScheme: false, // Don't respect OS preference, use our default
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Humanoid Robotics Logo',
        src: 'img/lllogo.png', // Placeholder, will be replaced with custom image
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar', // This will be adjusted once module pages are set up
          position: 'left',
          label: 'Textbook',
        },
        // Right-aligned items
        {
          type: 'localeDropdown', // Language switcher
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: '© 2025 Physical AI & Humanoid Robotics Textbook • Designed and developed ❤️ by Muntaha Noor',
      links: [
        {
          title: 'Social',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Muntaha-Noor/Physical-AI-Humanoid-Robotics-Textbook',
            },
            {
              label: 'ResearchGate',
              href: 'https://researchgate.net/',
            },
          ],
        },
        {
          title: ' ',
          items: [
            {
              html: '<div style="color: #b0b0c0; font-size: 0.9rem; line-height: 1.6;">Physical AI & Humanoid Robotics Research</div>',
            },
            {
              html: '<div style="color: #b0b0c0; font-size: 0.9rem; line-height: 1.6;">Simulation and ROS 2 Learning</div>',
            },
            {
              html: '<div style="color: #b0b0c0; font-size: 0.9rem; line-height: 1.6;">Vision-Language-Action Projects</div>',
            },
          ],
        },
      ],
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};
export default config;
