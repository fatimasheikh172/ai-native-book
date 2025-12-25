import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { text: inputValue, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Create a knowledge base based on the book content
      const knowledgeBase = [
        {
          keywords: ['physical ai', 'what is', 'definition', 'introduction'],
          response: "Physical AI represents the convergence of artificial intelligence with physical systems, particularly robotics. It's an interdisciplinary field combining machine learning, robotics, computer vision, natural language processing, and control systems to create intelligent agents that can perceive, reason, and act in the physical world."
        },
        {
          keywords: ['principles', 'core', 'foundational'],
          response: "Physical AI is built on several core principles: 1) Embodied Cognition: Intelligence emerges from interaction between an agent and its physical environment. 2) Sim-to-Real Transfer: Developing and testing AI in simulation before real-world deployment. 3) Perception-Action Loops: Continuous cycles of sensing, processing, and acting. 4) Safety-First Design: Prioritizing safety in all aspects."
        },
        {
          keywords: ['ros', 'ros 2', 'robot operating system', 'middleware'],
          response: "ROS 2 (Robot Operating System) is the middleware that enables communication between different components of robotic systems. It's part of the 'Robotic Nervous System' architecture that forms the foundation of physical AI systems."
        },
        {
          keywords: ['urdf', 'robot model', 'modeling'],
          response: "URDF (Unified Robot Description Format) is the standard for describing robot models and their kinematic structures. It's essential for robot modeling and simulation in physical AI systems."
        },
        {
          keywords: ['digital twin', 'simulation', 'unity', 'visualization'],
          response: "Digital twins are virtual representations of physical systems used for simulation and analysis. They include simulation environments, physics modeling, Unity integration for visualization, and model-based design and validation."
        },
        {
          keywords: ['ai brain', 'cognitive', 'planning', 'decision'],
          response: "The AI Robot Brain includes cognitive architectures for robotics, planning and decision-making systems, integration with NVIDIA Isaac and other AI frameworks, and learning and adaptation mechanisms."
        },
        {
          keywords: ['vla', 'vision language', 'human robot', 'interaction'],
          response: "Vision-Language-Action (VLA) systems enable human-robot interaction systems, voice-controlled navigation and manipulation, autonomous humanoid capabilities, and cognitive planning and reasoning."
        },
        {
          keywords: ['sensors', 'perception', 'sensor integration'],
          response: "Sensors are crucial components in robotics, providing the input needed for intelligent behavior. They enable perception systems and are integrated into robotic systems to enable sensing the environment."
        },
        {
          keywords: ['humanoid', 'robotics', 'human form', 'autonomous'],
          response: "Humanoid robotics involves creating robots that mimic human form and behavior for various applications. This includes autonomous humanoid capabilities and human-robot interaction systems."
        },
        {
          keywords: ['ai agents', 'autonomous', 'decision', 'behavior'],
          response: "AI agents in robotics make autonomous decisions based on sensor input and environmental conditions. They include planning and decision-making systems and learning and adaptation mechanisms."
        },
        {
          keywords: ['embodied', 'cognition', 'body', 'cognitive'],
          response: "Embodied Cognition is a core principle of Physical AI where intelligence emerges from the interaction between an agent and its physical environment. The body is not just a vessel but an integral part of the cognitive process."
        },
        {
          keywords: ['sim to real', 'simulation', 'transfer', 'real world'],
          response: "Sim-to-Real Transfer is the ability to develop and test AI algorithms in simulation environments before deploying them to real-world physical systems, ensuring safety and reducing costs."
        },
        {
          keywords: ['safety', 'design', 'first', 'first design'],
          response: "Safety-First Design prioritizes safety in all aspects of AI-physical system integration, from algorithm design to hardware implementation. This is a core principle of Physical AI."
        },
        {
          keywords: ['modules', 'curriculum', 'structure', 'learning'],
          response: "The curriculum has four modules: 1) The Robotic Nervous System (ROS 2, URDF, sensors), 2) The Digital Twin (simulation, Unity), 3) The AI Robot Brain (cognitive architectures), 4) Vision-Language-Action (human-robot interaction)."
        }
      ];

      // Simple keyword matching to find relevant response
      const userMessageLower = inputValue.toLowerCase();
      let foundResponse = null;

      for (const entry of knowledgeBase) {
        if (entry.keywords.some(keyword => userMessageLower.includes(keyword))) {
          foundResponse = entry.response;
          break;
        }
      }

      // Check if the question is about topics outside the book scope
      const outsideTopics = ['politics', 'unrelated', 'random', 'unrelated topic', 'different subject', 'not related', 'outside scope', 'unrelated question', 'not in book', 'not covered'];
      const isOutsideTopic = outsideTopics.some(topic => userMessageLower.includes(topic));

      // If no specific match found, provide a general response
      if (!foundResponse) {
        if (isOutsideTopic) {
          foundResponse = "I apologize, but I can only help with topics related to Physical AI and Human-Aided Robotics as covered in the book. I'm unable to answer questions outside the scope of the curriculum.";
        } else {
          foundResponse = "I'm the AI assistant for the Physical AI and Human-Aided Robotics book. I can help with questions about robotics, AI, ROS 2, digital twins, humanoid systems, and related topics. Could you ask about a specific concept from the book?";
        }
      }

      // Simulate thinking time
      await new Promise(resolve => setTimeout(resolve, 800));

      const botMessage = {
        text: foundResponse,
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      const botMessage = {
        text: 'I\'m here to help with questions about Physical AI and Human-Aided Robotics. Please ask about concepts like ROS 2, digital twins, humanoid robotics, or other topics from the book.',
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, botMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  if (!isOpen) {
    return (
      <div className={styles.chatContainer}>
        <button
          className={styles.chatButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          💬
        </button>
      </div>
    );
  }

  return (
    <div className={styles.chatContainer}>
      <div className={styles.chatWindow}>
        <div className={styles.chatHeader}>
          <span>AI Assistant</span>
          <button
            onClick={() => setIsOpen(false)}
            className={styles.closeButton}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>
        <div className={styles.chatMessages}>
          {messages.length === 0 ? (
            <div>Welcome! Ask me anything about the Physical AI and Human-Aided Robotics book.</div>
          ) : (
            messages.map((msg, index) => (
              <div
                key={index}
                className={`${styles.message} ${msg.sender === 'user' ? styles.userMessage : styles.botMessage}`}
              >
                {msg.text}
              </div>
            ))
          )}
          {isLoading && (
            <div className={`${styles.message} ${styles.botMessage}`}>
              Thinking...
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>
        <div className={styles.chatInput}>
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type your message..."
            disabled={isLoading}
          />
          <button onClick={sendMessage} disabled={isLoading}>
            Send
          </button>
        </div>
      </div>
    </div>
  );
};

export default ChatWidget;