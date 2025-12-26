// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Bridging the Digital Brain and Physical Protein',
  favicon: 'img/favicon.ico',

  // GitHub Pages Deployment
  url: 'https://fatimasheikh172.github.io',
  // NOTE: Local development ke liye '/' sahi hai, 
  // lekin GitHub deploy ke liye '/ai-native-book/' zaroori hai.
  baseUrl: '/', 

  organizationName: 'fatimasheikh172', 
  projectName: 'ai-native-book', 

  // CHANGED: Ise 'warn' par rakha hai taaki broken links ki wajah se build na ruke
  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  scripts: [
    {
      src: '/js/theme-toggle.js',
      async: true,
    },
  ],

  markdown: {
    mermaid: true,
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/fatimasheikh172/ai-native-book/tree/main/',
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {
            to: '/3d-explorer',
            label: '3D Explorer',
            position: 'left',
          },
          {
            href: 'https://github.com/fatimasheikh172/ai-native-book',
            label: 'GitHub',
            position: 'right',
          },
          {
            type: 'html',
            position: 'right',
            value: '<div id="theme-toggle-container"></div>',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Robotic Nervous System',
                to: '/docs/module-1-nervous-system/intro',
              },
              {
                label: 'The Digital Twin',
                to: '/docs/module-2-digital-twin/intro',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: '3D Visualization',
                to: '/docs/3d-visualization',
              },
              {
                label: 'AI Robot Brain',
                to: '/docs/module-3-ai-brain/intro',
              },
              {
                label: 'VLA System',
                to: '/docs/module-4-vla/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub Profile',
                href: 'https://github.com/fatimasheikh172',
              },
              {
                label: 'Docusaurus Discussions',
                href: 'https://github.com/facebook/docusaurus/discussions',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;