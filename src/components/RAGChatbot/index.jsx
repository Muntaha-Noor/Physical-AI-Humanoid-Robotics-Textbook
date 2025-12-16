import React, { useState, useEffect, useRef } from 'react';
import styles from './RAGChatbot.module.css';

const API_ENDPOINT = 'http://localhost:8000/query'; // Backend FastAPI endpoint

function RAGChatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]); // {id, type: 'user' | 'system', content: string | RetrievedChunk[], timestamp}
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(scrollToBottom, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!input.trim()) return;

    const userMessage = {
      id: Date.now(),
      type: 'user',
      content: input,
      timestamp: new Date()
    };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');
    setError(null);
    setIsLoading(true);

    try {
      const response = await fetch(API_ENDPOINT, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: userMessage.content, top_k: 3 }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      let systemContent;
      if (data.results && data.results.length > 0) {
        systemContent = data.results.map(chunk => ({
            text: chunk.text,
            source_url: chunk.source_url,
            chunk_id: chunk.chunk_id
        }));
      } else {
        systemContent = 'No relevant information found.';
      }

      const systemMessage = {
        id: Date.now() + 1,
        type: 'system',
        content: systemContent,
        timestamp: new Date()
      };
      setMessages((prevMessages) => [...prevMessages, systemMessage]);

    } catch (err) {
      console.error("API call failed:", err);
      setError(err.message || "An unknown error occurred. Please try again later.");
      const errorMessage = {
        id: Date.now() + 1,
        type: 'system',
        content: err.message || "An unknown error occurred. Please try again later.",
        timestamp: new Date()
      };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const renderMessageContent = (content) => {
    if (Array.isArray(content)) {
      return content.map((chunk, index) => (
        <div key={index} className={styles.chunk}>
          <div dangerouslySetInnerHTML={{ __html: chunk.text.replace(/\n/g, '<br />') }} />
          <a href={chunk.source_url} target="_blank" rel="noopener noreferrer">
            Source ({chunk.chunk_id})
          </a>
        </div>
      ));
    } else {
      // Improved formatting for paragraphs and lists
      const lines = content.split('\n');
      const elements = [];
      let currentList = null;
      let listItems = [];

      lines.forEach((line, index) => {
        if (line.trim().startsWith('- ') || line.trim().startsWith('* ')) {
          // This is a list item
          if (currentList === null) {
            currentList = 'ul';
            listItems = [];
          }
          const listItem = line.trim().substring(2); // Remove "- " or "* "
          listItems.push(<li key={`li-${index}`}>{listItem}</li>);
        } else if (line.trim() !== '') {
          // This is a paragraph or other content
          if (currentList !== null) {
            // Close the current list
            elements.push(<ul key={`list-${index}`}>{listItems}</ul>);
            currentList = null;
            listItems = [];
          }
          elements.push(<p key={`p-${index}`}>{line}</p>);
        } else if (currentList !== null && line.trim() === '') {
          // Empty line - close the current list
          elements.push(<ul key={`list-${index}`}>{listItems}</ul>);
          currentList = null;
          listItems = [];
        }
      });

      // Close any remaining list
      if (currentList !== null) {
        elements.push(<ul key={`list-end`}>{listItems}</ul>);
      }

      return elements;
    }
  };

  return (
    <>
      <button className={styles.chatButton} onClick={() => setIsOpen(!isOpen)}>
        {isOpen ? '✕' : '🤖'}
      </button>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>Humanoid AI Tutor</h3>
            <button onClick={() => setIsOpen(false)} className={styles.closeButton}>✕</button>
          </div>
          <div className={styles.messagesContainer}>
            {messages.map((msg) => (
              <div key={msg.id} className={`${styles.message} ${styles[msg.type]}`}>
                <div className={styles.messageContent}>
                  {renderMessageContent(msg.content)}
                </div>
                <div className={styles.timestamp}>
                  {msg.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.system}`}>
                <div className={styles.messageContent}>
                  <div className={styles.loadingDots}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            {error && (
              <div className={`${styles.message} ${styles.error}`}>
                <div className={styles.messageContent}>{error}</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question about Physical AI & Humanoid Robotics..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button type="submit" className={styles.sendButton} disabled={isLoading}>
              ➤
            </button>
          </form>
        </div>
      )}
    </>
  );
}

export default RAGChatbot;
