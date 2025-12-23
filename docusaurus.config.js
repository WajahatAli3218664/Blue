const { themes } = require('prism-react-renderer');
const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Textbook for Physical AI & Humanoid Robotics',
  favicon: 'img/logo.svg',

  url: 'https://bluebackend-sepia.vercel.app/',
  baseUrl: '/',

  organizationName: 'wajahatfrontdev-prog',
  projectName: 'book',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: false,

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  headTags: [
    {
      tagName: 'meta',
      attributes: {
        name: 'description',
        content:
          'Master Physical AI & Humanoid Robotics with ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models. Interactive AI-powered learning platform.',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        name: 'keywords',
        content:
          'Physical AI, Humanoid Robotics, ROS 2, NVIDIA Isaac, Gazebo, VLA, AI Robotics, Machine Learning',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'og:title',
        content: 'Physical AI & Humanoid Robotics - Interactive Textbook',
      },
    },
    {
      tagName: 'meta',
      attributes: {
        property: 'og:description',
        content:
          'Learn Physical AI & Humanoid Robotics with interactive AI chatbot, real-time Q&A, and comprehensive course modules.',
      },
    },
  ],

  plugins: [],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/docs',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    metadata: [
      { name: 'robots', content: 'index, follow' },
      { name: 'googlebot', content: 'index, follow' },
      { name: 'theme-color', content: '#3b82f6' }, // 🔵 BLUE
      {
        name: 'viewport',
        content: 'width=device-width, initial-scale=1.0, maximum-scale=5.0',
      },
    ],

    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },

    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Robot Logo',
        src: 'img/logo.svg',
        srcDark: 'img/logo-dark.svg',
        width: 40,
        height: 40,
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Course',
        },
        {
          to: '/docs/overview',
          label: 'Book Overview',
          position: 'left',
        },
        {
          href: 'https://github.com/wajahatfrontdev-prog/book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            { label: 'Book Overview', to: '/docs/overview' },
            { label: 'Module 1: ROS 2', to: '/docs/module1' },
            { label: 'Module 2: Gazebo & Unity', to: '/docs/module2' },
          ],
        },
        {
          title: 'Advanced',
          items: [
            { label: 'Module 3: NVIDIA Isaac', to: '/docs/module3' },
            { label: 'Module 4: VLA', to: '/docs/module4' },
            { label: 'Module 5: Capstone', to: '/docs/module5' },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/wajahatfrontdev-prog/book',
            },
            {
              label: 'Discussions',
              href: 'https://github.com/wajahatfrontdev-prog/book/discussions',
            },
          ],
        },
      ],

      copyright: `
      <div style="margin-top: 2rem; padding-top: 2rem; border-top: 2px solid #3b82f6;">
        <p style="color: #60a5fa; font-size: 1.1rem; font-weight: bold; margin-bottom: 0.5rem;">
          🤖 Physical AI & Humanoid Robotics
        </p>
        <p style="color: #cbd5f5; margin-bottom: 0.5rem;">
          Master the future of robotics with AI-powered learning
        </p>
        <p style="color: #94a3b8; font-size: 0.9rem;">
          Copyright © ${new Date().getFullYear()} | Built with ❤️ and AI
        </p>
        <p style="color: #60a5fa; font-size: 0.85rem; margin-top: 1rem;">
          🚀 Built with 
          <a href="https://github.com/panaversity/spec-kit-plus/" target="_blank" style="color:#60a5fa; text-decoration: underline;">
            Spec-Kit Plus
          </a> 
          & 
          <a href="https://www.claude.com/product/claude-code" target="_blank" style="color:#60a5fa; text-decoration: underline;">
            Claude Code
          </a>
        </p>
        <p style="color: #94a3b8; font-size: 0.8rem; margin-top: 0.5rem;">
          Created for 
          <a href="https://panaversity.org" target="_blank" style="color:#94a3b8; text-decoration: underline;">
            Panaversity
          </a> Hackathon I
        </p>
      </div>
      `,
    },

    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
    },
  },
};

module.exports = config;
