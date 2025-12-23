# Physical AI & Humanoid Robotics Textbook

An AI-Native textbook for teaching "Physical AI & Humanoid Robotics" course using Docusaurus, deployable to GitHub Pages, with integrated RAG chatbot.

## Features

- **Complete Course Content**: 5 comprehensive modules with 2 topics each covering Physical AI & Humanoid Robotics
- **Interactive Learning**: Integrated RAG (Retrieval-Augmented Generation) chatbot for real-time Q&A
- **Text Selection**: Click-to-ask functionality for contextual questions
- **Responsive Design**: Mobile-first design with dark/light mode support
- **Progress Tracking**: Built-in progress tracking and reading analytics
- **Module Previews**: Engaging course preview section on homepage
- **Personalization**: Customizable display options (font size, theme, animations)
- **Multilingual Support**: Urdu translation capability
- **Hardware Integration**: RTX workstation, Jetson Orin Nano, RealSense sensor notes
- **Cloud Options**: AWS RoboMaker and NVIDIA Omniverse Cloud integration guidance

## Tech Stack

- **Framework**: Docusaurus 3.0
- **Frontend**: React 18
- **Backend**: FastAPI (Python)
- **Database**: Qdrant (vector database), Neon Postgres
- **AI**: OpenAI API for RAG chatbot and cognitive planning
- **Deployment**: GitHub Pages

## Getting Started

### Prerequisites

- Node.js 18+
- npm or yarn
- Python 3.9+ (for backend services)
- Git

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd physical-ai-humanoid-robotics-textbook
   ```

2. Install frontend dependencies:
   ```bash
   npm install
   ```

3. Set up backend (for RAG chatbot):
   ```bash
   cd api/rag-chatbot
   pip install -r requirements.txt
   ```

4. Create environment variables file in the project root:
   ```env
   OPENAI_API_KEY=your_openai_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   NEON_DB_URL=your_neon_db_url
   ```

### Running the Application

1. For development:
   ```bash
   # Terminal 1: Start the Docusaurus development server
   npm run start

   # Terminal 2: Start the backend API server
   cd api/rag-chatbot
   python main.py
   ```

2. For production build:
   ```bash
   # Build the static site
   npm run build

   # Serve the build locally
   npm run serve
   ```

## Course Content Structure

The textbook is organized into 5 comprehensive modules:

- **Module 1**: The Robotic Nervous System (ROS 2)
  - ROS 2 Nodes & Topics
  - URDF & Python Agent Integration

- **Module 2**: The Digital Twin (Gazebo & Unity)
  - Physics Simulation
  - Sensors & Environment Rendering

- **Module 3**: The AI-Robot Brain (NVIDIA Isaac)
  - Isaac ROS Perception
  - Path Planning & Navigation

- **Module 4**: Vision-Language-Action (VLA)
  - Voice-to-Action
  - Cognitive Planning with LLMs

- **Module 5**: Capstone Autonomous Humanoid Project
  - Simulated Humanoid
  - Obstacle Navigation

## Key Features

### Interactive Learning
The integrated RAG chatbot allows students to select text and ask questions about the content in real-time, with responses contextualized to the selected material.

### Personalization
Users can customize their learning experience with options for:
- Font size adjustment
- Theme selection (light/dark/auto)
- Animation preferences
- Reading speed settings
- Progress tracking visibility

### Multilingual Support
Content translation capabilities with initial Urdu support, allowing broader accessibility.

### Hardware Integration Notes
Practical notes throughout the content about:
- RTX-enabled workstations for intensive computation
- Jetson Orin Nano for edge deployment
- RealSense sensors for perception
- Cloud options (AWS RoboMaker, NVIDIA Omniverse Cloud)
- Latency management strategies

## Deployment

The site is configured for GitHub Pages deployment. The static site will be automatically deployed when pushed to the main branch.

To deploy to GitHub Pages:
1. Push changes to the main branch
2. Enable GitHub Pages in repository settings (source: gh-pages branch)
3. The site will be available at `https://<username>.github.io/<repository>`

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the MIT License.# Physical-AI-Humanoid-Robotics-Book
