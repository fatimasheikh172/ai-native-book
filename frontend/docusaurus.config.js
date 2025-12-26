// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Human-Aided Robotics',
  tagline: 'Bracing the Digital Brain and Physical Protein',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://fatimasheikh172.github.io',
  
  // --- CHANGED THIS SECTION ---
  // Changed from '/ai-native-book' to '/' for local development
  baseUrl: '/', 
  // ----------------------------

  // GitHub pages deployment config.
  organizationName: 'fatimasheikh172', 
  projectName: 'ai-native-book', 

  onBrokenLinks: 'throw',
  
  // Moved scripts here (Root level) so they actually work
  scripts: [
    {
      src: '/js/theme-toggle.js',
      async: true,
    },
  ],

  markdown: {
    mermaid: true,
    parseFrontMatter: undefined,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
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
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
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
            href: 'https://github.com/facebook/docusaurus',
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
            title: 'Academics',
            items: [
              {
                label: 'Introduction',
                to: '/docs/module-1-nervous-system/intro',
              },
              {
                label: 'Course Modules',
                to: '/docs/category/modules',
              },
              {
                label: 'Research Papers',
                to: '/docs/category/research',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Documentation',
                to: '/docs/intro',
              },
              {
                label: 'Tutorials',
                to: '/docs/category/tutorials',
              },
              {
                label: 'Examples',
                to: '/docs/category/examples',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/fatimasheikh172/ai-native-book',
              },
              {
                label: 'Discussions',
                href: 'https://github.com/facebook/docusaurus/discussions',
              },
              {
                label: 'Contributing',
                to: '/docs/contributing',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Human-Aided Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;