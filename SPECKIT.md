# Speckit Integration

This project is powered by **Speckit AI** - an advanced AI platform for building intelligent applications.

## Features

### Frontend Integration
- **Speckit Widget**: Integrated in the homepage for enhanced user experience
- **Theme Customization**: Matches the book's neon green and black theme
- **Position**: Bottom-left corner for easy access

### Backend Integration
- **API Branding**: All API responses include Speckit attribution
- **Analytics**: Track user interactions and chatbot usage
- **Performance Monitoring**: Monitor API response times and errors

## Configuration

### Environment Variables

Create a `.env` file in the root directory:

```env
REACT_APP_SPECKIT_API_KEY=your_speckit_api_key
SPECKIT_API_KEY=your_speckit_api_key
GROQ_API_KEY=your_groq_api_key
```

### Frontend Setup

The Speckit component is automatically loaded on all pages:

```javascript
// src/components/Speckit.js
window.speckitConfig = {
  apiKey: process.env.REACT_APP_SPECKIT_API_KEY,
  theme: 'dark',
  position: 'bottom-left',
  primaryColor: '#00ff41',
};
```

### Backend Setup

The FastAPI backend includes Speckit branding:

```python
# api/rag-chatbot/main.py
app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot API (Powered by Speckit)",
    version="1.0.0"
)
```

## Benefits

1. **Enhanced Analytics**: Track user engagement and learning patterns
2. **AI-Powered Insights**: Get insights into common questions and topics
3. **Performance Monitoring**: Monitor chatbot performance and response quality
4. **User Feedback**: Collect feedback to improve the learning experience

## Usage

### In the Book
- Speckit widget appears on all pages
- Users can access additional AI features
- Seamless integration with existing chatbot

### In the Chatbot
- All API responses include Speckit attribution
- Analytics track conversation patterns
- Performance metrics for optimization

## Speckit Features Used

- ✅ Widget Integration
- ✅ Theme Customization
- ✅ API Branding
- ✅ Analytics Tracking
- ✅ Performance Monitoring

## Support

For Speckit-related issues or questions:
- Visit: https://speckit.ai
- Documentation: https://docs.speckit.ai
- Support: support@speckit.ai

---

**Powered by Speckit AI** 🚀
