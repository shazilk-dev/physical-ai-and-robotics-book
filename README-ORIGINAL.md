# Physical AI & Humanoid Robotics Textbook

[![Deploy Frontend](https://github.com/shazilk-dev/physical-ai-and-robotics-book/actions/workflows/deploy-frontend.yml/badge.svg)](https://github.com/shazilk-dev/physical-ai-and-robotics-book/actions/workflows/deploy-frontend.yml)

An interactive, open-source textbook covering Physical AI, Humanoid Robotics, ROS 2, NVIDIA Isaac Sim, and Vision-Language-Action models.

## 🎯 Features

- **📚 Comprehensive Content**: 4 modules covering ROS 2, robot architecture, simulation (Isaac/Gazebo), and VLA models
- **🤖 RAG Chatbot**: Interactive AI assistant powered by OpenAI and Qdrant vector database
- **👤 Personalization**: Adaptive content based on user background (beginner/advanced)
- **🌍 Translation**: English and Urdu language support
- **🧪 Hands-on Labs**: 8 practical labs with starter code, solutions, and automated grading
- **🎓 Capstone Project**: Voice-to-action humanoid robot demo

## 📁 Project Structure

```
physical-ai-humanoids-textbook/
├── backend/           # FastAPI RAG chatbot backend
├── database/          # PostgreSQL schemas and migrations
├── auth/              # Better-auth authentication
├── frontend/          # Docusaurus textbook content
├── labs/              # Hands-on lab exercises
├── ros2_packages/     # ROS 2 workspace
├── isaac_assets/      # NVIDIA Isaac Sim assets
├── hardware/          # Hardware setup guides (Jetson, sensors)
├── cloud/             # Cloud deployment configs (AWS, Azure, GCP)
├── scripts/           # Automation scripts
└── grading/           # Automated grading infrastructure
```

## 🚀 Quick Start

### Prerequisites

- **Node.js** 20+
- **Python** 3.11+
- **Docker** (optional, for containerized deployment)
- **ROS 2 Humble** (for labs)
- **NVIDIA Jetson** (optional, for edge deployment)

### Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/shazilk-dev/physical-ai-and-robotics-book.git
   cd physical-ai-and-robotics-book
   ```

2. **Install frontend dependencies**

   ```bash
   cd docs
   npm install
   ```

3. **Install backend dependencies**

   ```bash
   cd ../backend
   pip install -r requirements.txt
   cp .env.example .env
   # Edit .env with your API keys
   ```

4. **Run locally**

   ```bash
   # Terminal 1: Frontend
   npm run dev

   # Terminal 2: Backend
   npm run backend
   ```

5. **Access the textbook**
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000
   - API Docs: http://localhost:8000/docs

### Docker Deployment

```bash
docker-compose up -d
```

## 📖 Content Modules

### Module 1: ROS 2 Fundamentals

- ROS 2 architecture, rclpy, launch files
- URDF robot descriptions
- Sensors and proprioception

### Module 2: Robot Architecture

- Kinematics and dynamics
- Actuation systems (electric, hydraulic, QDD)
- Edge compute and perception (Jetson Orin/Thor)

### Module 3: Simulation & Sim-to-Real

- NVIDIA Isaac Sim + ROS integration
- Gazebo, MuJoCo, Unity
- Domain randomization and policy transfer

### Module 4: VLA Models & Voice-to-Action

- Vision-Language-Action architectures
- OpenAI Whisper integration
- Capstone: Voice-controlled humanoid demo

## 🧪 Labs

1. **Lab 1**: ROS 2 Basics - Heartbeat node
2. **Lab 2**: URDF Humanoid Description
3. **Lab 3**: Gazebo Simulation
4. **Lab 4**: Isaac Sim Setup
5. **Lab 5**: Isaac ROS Visual SLAM
6. **Lab 6**: Nav2 Integration
7. **Lab 7**: Voice-to-Action Pipeline
8. **Lab 8**: Sim-to-Real Transfer

## 🤖 RAG Chatbot

The textbook includes an AI assistant that:

- Answers questions about content
- Provides code examples
- Explains concepts based on user background
- Supports text selection queries

**Technologies:**

- **Backend**: FastAPI + Qdrant + OpenAI
- **Database**: Neon Postgres
- **Embeddings**: text-embedding-3-small
- **LLM**: GPT-4o-mini

## 🔐 Authentication

Uses **Better-auth** with:

- Email/password authentication
- OAuth providers (Google, GitHub)
- User background survey for personalization

## 🌍 Translation

- **English** (default)
- **Urdu** (AI-powered translation via OpenAI)

## 📊 Tech Stack

| Layer          | Technology                                   |
| -------------- | -------------------------------------------- |
| **Frontend**   | Docusaurus, React, TypeScript                |
| **Backend**    | FastAPI, Python                              |
| **Database**   | Neon Postgres (Serverless)                   |
| **Vector DB**  | Qdrant Cloud                                 |
| **AI**         | OpenAI (GPT-4o-mini, text-embedding-3-small) |
| **Auth**       | Better-auth                                  |
| **Deployment** | GitHub Pages (frontend), Render (backend)    |
| **CI/CD**      | GitHub Actions                               |

## 🛠️ Development

### Running Tests

```bash
# Backend tests
cd backend
pytest

# Frontend build test
cd frontend
npm run build
```

### Seeding Vector Database

```bash
python scripts/rag/seed-vector-db.py
```

### Adding New Content

1. Add markdown files to `frontend/docs/XX-module-Y/`
2. Update `frontend/sidebars.ts`
3. Run `npm run build` to test
4. Seed embeddings: `python scripts/rag/seed-vector-db.py`

## 📝 License

MIT License - See [LICENSE](LICENSE) for details

## 🤝 Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## 📧 Contact

- **Author**: shazilk-dev
- **GitHub**: [@shazilk-dev](https://github.com/shazilk-dev)

## 🙏 Acknowledgments

- NVIDIA Isaac Sim team
- ROS 2 community
- OpenAI for embeddings and LLMs
- Qdrant for vector search

---

**⭐ Star this repo if you find it helpful!**
