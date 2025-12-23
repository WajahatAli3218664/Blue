import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import ChatBot from '../components/ChatBot';
import Speckit from '../components/Speckit';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <p className={styles.heroDescription}>
          Master the future of robotics with hands-on learning in ROS 2, AI perception, 
          and autonomous systems. Build real humanoid robots from simulation to deployment.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning - 5min ⏱️
          </Link>
          <Link
            className="button button--outline button--lg"
            to="/docs/module1"
            style={{marginLeft: '1rem'}}>
            View Modules 📚
          </Link>
        </div>
        <div className={styles.stats}>
          <div className={styles.stat}>
            <div className={styles.statNumber}>5</div>
            <div className={styles.statLabel}>Modules</div>
          </div>
          <div className={styles.stat}>
            <div className={styles.statNumber}>10</div>
            <div className={styles.statLabel}>Topics</div>
          </div>
          <div className={styles.stat}>
            <div className={styles.statNumber}>AI</div>
            <div className={styles.statLabel}>Powered</div>
          </div>
        </div>
      </div>
    </header>
  );
}

const ModuleList = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Master ROS 2 nodes, topics, and URDF integration for robotic communication systems.',
    link: '/docs/module1',
    icon: '🧠',
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: 'Create realistic physics simulations and sensor environments for robot development.',
    link: '/docs/module2',
    icon: '🌐',
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    description: 'Implement advanced perception, SLAM, and navigation using GPU-accelerated AI.',
    link: '/docs/module3',
    icon: '🚀',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Integrate natural language processing with robotic actions using LLMs.',
    link: '/docs/module4',
    icon: '🗣️',
  },
  {
    title: 'Module 5: Capstone Autonomous Humanoid Project',
    description: 'Build a complete autonomous humanoid robot with advanced navigation.',
    link: '/docs/module5',
    icon: '🤖',
  },
];

function Module({title, description, link, icon}) {
  return (
    <div className={clsx('col col--4', styles.moduleCard)}>
      <div className="text--center">
        <div className={styles.moduleIcon}>{icon}</div>
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
        <Link
          className="button button--primary button--sm"
          to={link}>
          Start Module
        </Link>
      </div>
    </div>
  );
}

function HomepageModules() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h2 className="text--center margin-bottom--lg">Course Modules</h2>
          </div>
        </div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h2 className="text--center margin-bottom--lg">Why Choose This Course?</h2>
          </div>
        </div>
        <div className="row">
          <div className="col col--3">
            <div className={styles.featureCard}>
              <div className={styles.featureIcon}>🤖</div>
              <h3>AI-Powered Chatbot</h3>
              <p>Select any text and ask questions. Get instant explanations and clarifications.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.featureCard}>
              <div className={styles.featureIcon}>✨</div>
              <h3>Interactive Learning</h3>
              <p>Hands-on projects with real-world applications and simulations.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.featureCard}>
              <div className={styles.featureIcon}>📊</div>
              <h3>Progress Tracking</h3>
              <p>Monitor your learning journey with built-in analytics and milestones.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.featureCard}>
              <div className={styles.featureIcon}>🚀</div>
              <h3>Industry Ready</h3>
              <p>Learn tools used by top robotics companies worldwide.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="An AI-Native textbook for Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageModules />
        <HomepageFeatures />
        <TechStack />
        <LearningPath />
      </main>
      <ChatBot />
      <Speckit />
    </Layout>
  );
}

function TechStack() {
  return (
    <section className={styles.techStack}>
      <div className="container">
        <h2 className="text--center margin-bottom--lg">Technologies You'll Master</h2>
        <div className="row">
          <div className="col col--4">
            <div className={styles.techCard}>
              <h3>🤖 ROS 2</h3>
              <p>Robot Operating System for building distributed robotic applications</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.techCard}>
              <h3>🎮 Gazebo & Unity</h3>
              <p>Physics simulation and digital twin environments</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.techCard}>
              <h3>⚡ NVIDIA Isaac</h3>
              <p>GPU-accelerated AI perception and navigation</p>
            </div>
          </div>
        </div>
        <div className="row" style={{marginTop: '2rem'}}>
          <div className="col col--4">
            <div className={styles.techCard}>
              <h3>🧠 Vision-Language-Action</h3>
              <p>LLM-powered cognitive planning and voice control</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.techCard}>
              <h3>📷 RealSense</h3>
              <p>Depth sensing and 3D perception</p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.techCard}>
              <h3>☁️ Cloud Platforms</h3>
              <p>AWS RoboMaker & NVIDIA Omniverse</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function LearningPath() {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        <h2 className="text--center margin-bottom--lg">Your Learning Journey</h2>
        <div className={styles.pathContainer}>
          <div className={styles.pathStep}>
            <div className={styles.pathNumber}>1</div>
            <h3>Foundation</h3>
            <p>Learn ROS 2 basics, nodes, topics, and URDF modeling</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.pathNumber}>2</div>
            <h3>Simulation</h3>
            <p>Master Gazebo physics and Unity rendering</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.pathNumber}>3</div>
            <h3>AI Integration</h3>
            <p>Implement NVIDIA Isaac perception and navigation</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.pathNumber}>4</div>
            <h3>Advanced AI</h3>
            <p>Build VLA systems with LLM cognitive planning</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.pathNumber}>5</div>
            <h3>Capstone</h3>
            <p>Create autonomous humanoid robot project</p>
          </div>
        </div>
      </div>
    </section>
  );
}