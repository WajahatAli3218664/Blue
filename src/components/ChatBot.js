import React, { useState, useEffect, useRef } from 'react';
import './ChatBot.css';

const ChatBot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { type: 'bot', content: 'Hello! I\'m your AI assistant for Physical AI & Humanoid Robotics. Select any text on the page and ask me questions!' }
  ]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    // Handle text selection
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      
      if (text.length > 0) {
        setSelectedText(text);
        
        // Create ask button
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        
        // Remove existing ask buttons
        document.querySelectorAll('.ask-button').forEach(btn => btn.remove());
        
        // Create new ask button
        const askButton = document.createElement('button');
        askButton.className = 'ask-button';
        askButton.textContent = 'Ask AI';
        askButton.style.position = 'fixed';
        askButton.style.top = `${rect.top - 40}px`;
        askButton.style.left = `${rect.left + rect.width / 2 - 30}px`;
        askButton.style.zIndex = '1001';
        
        askButton.onclick = () => {
          setIsOpen(true);
          setInputMessage(`About "${text}": `);
          askButton.remove();
          selection.removeAllRanges();
        };
        
        document.body.appendChild(askButton);
        
        // Remove button after 5 seconds
        setTimeout(() => {
          if (document.body.contains(askButton)) {
            askButton.remove();
          }
        }, 5000);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.querySelectorAll('.ask-button').forEach(btn => btn.remove());
    };
  }, []);

  const sendMessage = async () => {
    if (!inputMessage.trim() || isLoading) return;

    const userMessage = inputMessage;
    setInputMessage('');
    setMessages(prev => [...prev, { type: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      // Get API URL from environment or use fallback
      const baseUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_API_URL) 
        ? process.env.REACT_APP_API_URL 
        : 'https://book-backend-33x5ci25c-wajahatfrontdev-8765s-projects.vercel.app';
      const apiUrl = `${baseUrl}/api/chat`;
      
      const response = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage,
          context: selectedText
        }),
      });

      const data = await response.json();
      setMessages(prev => [...prev, { type: 'bot', content: data.response || 'Sorry, no response received.' }]);
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => [...prev, { type: 'bot', content: 'Connection error. Please check if the backend API is configured correctly.' }]);
    } finally {
      setIsLoading(false);
      setSelectedText('');
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="chatbot-container">
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <span>AI Assistant</span>
            <button 
              className="chatbot-close" 
              onClick={() => setIsOpen(false)}
            >
              ×
            </button>
          </div>
          
          <div className="chatbot-messages">
            {messages.map((message, index) => (
              <div key={index} className={`message ${message.type}`}>
                {message.content}
              </div>
            ))}
            {isLoading && (
              <div className="message bot">
                <span className="loading-dots">Thinking</span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          
          <div className="chatbot-input-container">
            <input
              type="text"
              className="chatbot-input"
              placeholder="Ask me anything about the content..."
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              onKeyPress={handleKeyPress}
              disabled={isLoading}
            />
            <button 
              className="chatbot-send" 
              onClick={sendMessage}
              disabled={isLoading || !inputMessage.trim()}
            >
              Send
            </button>
          </div>
        </div>
      )}
      
      <button 
        className="chatbot-toggle" 
        onClick={() => setIsOpen(!isOpen)}
        title="AI Assistant"
      >
        🤖
      </button>
    </div>
  );
};

export default ChatBot;