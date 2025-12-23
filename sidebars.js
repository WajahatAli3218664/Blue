const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/index',
        'module1/nodes-topics',
        'module1/urdf-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/index',
        'module2/physics-simulation',
        'module2/sensors-rendering',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/index',
        'module3/isaac-perception',
        'module3/path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/index',
        'module4/voice-to-action',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone Autonomous Humanoid Project',
      items: [
        'module5/index',
        'module5/simulated-humanoid',
        'module5/obstacle-navigation',
      ],
    },
  ],
};

module.exports = sidebars;