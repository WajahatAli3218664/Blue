# Voice-to-Action

## Introduction

Voice-to-Action systems enable robots to understand and execute natural language commands through speech recognition and action mapping.

## Speech Recognition Setup

```python
import rclpy
from rclpy.node import Node
import speech_recognition as sr
import pyttsx3
from geometry_msgs.msg import Twist
import threading

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Text-to-speech
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        
        # Voice commands mapping
        self.commands = {
            'move forward': self.move_forward,
            'move backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'go to kitchen': lambda: self.navigate_to_location('kitchen'),
            'go to bedroom': lambda: self.navigate_to_location('bedroom'),
        }
        
        # Start listening thread
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        self.listening_thread.start()
        
        self.get_logger().info('Voice control node started. Say commands!')
    
    def listen_continuously(self):
        """Continuously listen for voice commands"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        
        while rclpy.ok():
            try:
                with self.microphone as source:
                    # Listen for audio with timeout
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                
                # Recognize speech
                command = self.recognizer.recognize_google(audio).lower()
                self.get_logger().info(f'Heard: "{command}"')
                
                # Process command
                self.process_command(command)
                
            except sr.WaitTimeoutError:
                pass  # No speech detected
            except sr.UnknownValueError:
                self.speak("Sorry, I didn't understand that.")
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition error: {e}')
    
    def process_command(self, command):
        """Process recognized voice command"""
        # Find matching command
        for key, action in self.commands.items():
            if key in command:
                self.speak(f"Executing: {key}")
                action()
                return
        
        # If no direct match, try LLM interpretation
        self.interpret_with_llm(command)
    
    def interpret_with_llm(self, command):
        """Use LLM to interpret complex commands"""
        # This would integrate with your LLM API
        prompt = f"""
        Convert this natural language command to robot action:
        Command: "{command}"
        
        Available actions:
        - move_forward()
        - move_backward()
        - turn_left()
        - turn_right()
        - stop_robot()
        - navigate_to_location(location)
        
        Return the function call:
        """
        
        # Call LLM API here
        # For now, simple fallback
        self.speak("I'm not sure how to do that yet.")
    
    def speak(self, text):
        """Convert text to speech"""
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()
    
    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)
        
    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.5
        self.cmd_vel_pub.publish(cmd)
        
    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.angular.z = 0.5
        self.cmd_vel_pub.publish(cmd)
        
    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.angular.z = -0.5
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
    def navigate_to_location(self, location):
        """Navigate to specific location"""
        self.get_logger().info(f'Navigating to {location}')
        # Integrate with navigation stack

def main(args=None):
    rclpy.init(args=args)
    voice_control = VoiceControlNode()
    rclpy.spin(voice_control)
    voice_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Natural Language Processing

```python
import openai
from transformers import pipeline

class AdvancedNLP:
    def __init__(self):
        # Initialize models
        self.intent_classifier = pipeline("text-classification", 
                                         model="microsoft/DialoGPT-medium")
        self.ner_model = pipeline("ner", 
                                model="dbmdz/bert-large-cased-finetuned-conll03-english")
    
    def extract_intent_and_entities(self, command):
        """Extract intent and entities from command"""
        # Intent classification
        intent = self.classify_intent(command)
        
        # Named entity recognition
        entities = self.ner_model(command)
        
        return intent, entities
    
    def classify_intent(self, command):
        """Classify the intent of the command"""
        intents = {
            'navigation': ['go', 'move', 'navigate', 'walk'],
            'manipulation': ['pick', 'grab', 'place', 'put'],
            'observation': ['look', 'see', 'find', 'search'],
            'interaction': ['say', 'tell', 'ask', 'speak']
        }
        
        command_lower = command.lower()
        for intent, keywords in intents.items():
            if any(keyword in command_lower for keyword in keywords):
                return intent
        
        return 'unknown'
```

## Hardware Integration

- **Edge Deployment**: Optimize speech recognition for Jetson Orin Nano
- **Cloud Processing**: Use AWS Transcribe for advanced speech recognition
- **Real-time Performance**: Balance accuracy with latency requirements