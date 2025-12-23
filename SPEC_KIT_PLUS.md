# Spec-Kit Plus Integration

This project is built using **Spec-Kit Plus** - A powerful AI-driven development toolkit for creating intelligent applications.

## Project Requirements Met

### ✅ Core Requirements (100 Points)

1. **AI/Spec-Driven Book Creation**
   - ✅ Built with Docusaurus
   - ✅ Deployed to GitHub Pages
   - ✅ Created using Spec-Kit Plus and Claude Code
   - ✅ 5 comprehensive modules with 10 topics

2. **Integrated RAG Chatbot**
   - ✅ FastAPI backend with RAG capabilities
   - ✅ OpenAI/Groq API integration
   - ✅ Text selection Q&A feature
   - ✅ Context-aware responses
   - ✅ Neon Postgres ready (configured)
   - ✅ Qdrant Cloud ready (configured)

### 🎯 Bonus Features (Up to 200 Extra Points)

#### Bonus 1: Claude Code Subagents & Skills (50 points)
- ✅ Reusable components created
- ✅ Modular architecture
- ✅ AI-driven development workflow

#### Bonus 2: Better-Auth Signup/Signin (50 points)
- 🔄 Ready for implementation
- 📝 User background questionnaire planned
- 🎯 Personalization based on user profile

#### Bonus 3: Content Personalization (50 points)
- 🔄 Personalization button at chapter start
- 🎯 Adjust content based on user background
- 📊 Software/Hardware experience levels

#### Bonus 4: Urdu Translation (50 points)
- 🔄 Translation button at chapter start
- 🌐 Real-time content translation
- 📚 Multilingual support

## Spec-Kit Plus Usage

### Development Process
All code in this project was developed using:
- **Spec-Kit Plus**: For AI-driven specifications and requirements
- **Claude Code**: For intelligent code generation
- **Docusaurus**: For book framework
- **FastAPI**: For backend API

### Key Features Built with Spec-Kit Plus

1. **Book Structure**
   - 5 modules covering Physical AI & Humanoid Robotics
   - Interactive learning with AI chatbot
   - Progress tracking
   - Responsive design

2. **RAG Chatbot**
   - Text selection Q&A
   - Context-aware responses
   - Real-time interaction
   - FastAPI backend

3. **Theme & Design**
   - Mountain Dew inspired (black & neon green)
   - Dark/Light mode support
   - Mobile responsive
   - Smooth animations

## Technologies Stack

- **Frontend**: React 18, Docusaurus 3.0
- **Backend**: FastAPI (Python)
- **Database**: Neon Postgres (Serverless)
- **Vector DB**: Qdrant Cloud (Free Tier)
- **AI**: OpenAI API / Groq API
- **Deployment**: GitHub Pages
- **Development**: Spec-Kit Plus + Claude Code

## Course Content

### Module 1: The Robotic Nervous System (ROS 2)
- ROS 2 Nodes & Topics
- URDF & Python Agent Integration

### Module 2: The Digital Twin (Gazebo & Unity)
- Physics Simulation
- Sensors & Environment Rendering

### Module 3: The AI-Robot Brain (NVIDIA Isaac)
- Isaac ROS Perception
- Path Planning & Navigation

### Module 4: Vision-Language-Action (VLA)
- Voice-to-Action
- Cognitive Planning with LLMs

### Module 5: Capstone Autonomous Humanoid Project
- Simulated Humanoid
- Obstacle Navigation

## Setup Instructions

### Prerequisites
```bash
Node.js 18+
Python 3.9+
Git
```

### Installation
```bash
# Clone repository
git clone <repository-url>
cd physical-ai-humanoid-robotics-textbook

# Install dependencies
npm install

# Setup backend
cd api/rag-chatbot
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Add your API keys
```

### Environment Variables
```env
GROQ_API_KEY=your_groq_api_key
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_db_url
```

### Run Development
```bash
# Terminal 1: Frontend
npm start

# Terminal 2: Backend
cd api/rag-chatbot
python main.py
```

### Deploy to GitHub Pages
```bash
npm run build
npm run deploy
```

## Hackathon Compliance

### Core Deliverables ✅
- [x] Docusaurus book with 5 modules
- [x] GitHub Pages deployment
- [x] RAG chatbot with FastAPI
- [x] Text selection Q&A feature
- [x] OpenAI/Groq integration
- [x] Neon Postgres configuration
- [x] Qdrant Cloud configuration

### Bonus Features 🎯
- [x] Spec-Kit Plus usage documented
- [x] Claude Code development workflow
- [x] Reusable components
- [ ] Better-Auth integration (ready)
- [ ] User personalization (ready)
- [ ] Urdu translation (ready)

## Credits

**Built with:**
- Spec-Kit Plus (https://github.com/panaversity/spec-kit-plus/)
- Claude Code (https://www.claude.com/product/claude-code)
- Panaversity Initiative (https://panaversity.org)

**For:**
- Physical AI & Humanoid Robotics Course
- Panaversity Hackathon I
- AI-Native Technical Textbooks

---

**Powered by Spec-Kit Plus & Claude Code** 🚀
**Created for Panaversity Hackathon** 🎯
