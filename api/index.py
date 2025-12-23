from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
import requests
import os

GROQ_API_KEY = os.getenv("GROQ_API_KEY", "")

app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot API",
    description="Built with Spec-Kit Plus & Qwen Code for Panaversity Hackathon I",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    message: str
    context: str = ""

class ChatResponse(BaseModel):
    response: str
    sources: List[str] = []

async def call_groq_api(message: str, context: str = "") -> str:
    try:
        if not GROQ_API_KEY:
            return "Please configure GROQ_API_KEY in environment variables"

        headers = {
            "Authorization": f"Bearer {GROQ_API_KEY}",
            "Content-Type": "application/json"
        }

        system_prompt = """You are a friendly AI assistant for Physical AI & Humanoid Robotics.

Help with ROS 2, Gazebo, NVIDIA Isaac, VLA models, and humanoid robotics.
Be concise and helpful."""

        user_message = message
        if context:
            user_message = f"Based on: '{context}'\n\nQuestion: {message}"

        data = {
            "model": "llama-3.1-8b-instant",
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ],
            "temperature": 0.7,
            "max_tokens": 800
        }

        response = requests.post(
            "https://api.groq.com/openai/v1/chat/completions",
            headers=headers,
            json=data,
            timeout=15
        )

        if response.status_code == 200:
            result = response.json()
            return result["choices"][0]["message"]["content"]
        else:
            return "I'm having trouble right now. Please try again."

    except Exception as e:
        return "Sorry, I'm having connection issues. Please try again."

@app.get("/")
async def root():
    return {
        "message": "Physical AI & Humanoid Robotics Chatbot API is running",
        "status": "healthy",
        "built_with": "Spec-Kit Plus & Claude Code",
        "hackathon": "Panaversity Hackathon I",
        "features": ["RAG", "LLM Integration", "Text Selection Q&A", "Context-Aware Responses"],
        "github": "https://github.com/panaversity/spec-kit-plus/"
    }

@app.get("/api/health")
async def health():
    return {"status": "healthy", "service": "ai-chatbot"}

@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    try:
        response = await call_groq_api(request.message, request.context)
        return ChatResponse(response=response, sources=[])
    except Exception as e:
        return ChatResponse(response="Sorry, I'm having issues right now.", sources=[])
