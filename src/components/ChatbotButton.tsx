import React, { useState, useEffect, useRef } from 'react';
import styles from './ChatbotButton.module.css';

// Define the structure for a single message
interface Message {
  sender: 'user' | 'bot';
  text: string;
}

// The backend is expected to return this shape
interface ApiResponse {
  query: string;
  results: Array<{
    text: string;
    source_url: string;
    chunk_id: number;
    score: number;
  }>;
}

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const API_URL = 'http://localhost:8000/query';

  // Effect to scroll to the latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
        // Add a welcome message when opening the chat for the first time
        if (messages.length === 0) {
            setMessages([{ sender: 'bot', text: "Hello! Ask me anything about the textbook." }]);
        }
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = { sender: 'user', text: inputValue };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch(API_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: inputValue, top_k: 3 }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data: ApiResponse = await response.json();

      let botText = '';
      if (data.results && data.results.length > 0) {
        // Process each result to create ChatGPT-style formatting
        const processedResults = data.results.map((result, index) => {
          let content = result.text;

          // Identify and create headings for major sections
          content = content.replace(/^([A-Z][^:]*?):\s*\n/gm, '## $1\n');
          content = content.replace(/^(Module \d+:[^\n]*)/gm, '## $1');
          content = content.replace(/^(What You Will Learn)/gm, '## $1');
          content = content.replace(/^(Throughout this course)/gm, '### $1');
          content = content.replace(/^- (Module [^:\n]*:)/gm, '### $1');

          // Convert bullet points to proper markdown format
          content = content.replace(/^\s*- /gm, '- ');

          // Add source reference at the end
          if (result.source_url) {
            const sourceName = (new URL(result.source_url)).pathname.split('/').pop() || 'source';
            content += `\n\n**Source:** [${sourceName}](${result.source_url})`;
          }

          return content.trim();
        });

        // Join the formatted results
        botText = processedResults.join('\n\n---\n\n');
      } else {
        botText = 'No relevant information found.';
      }

      const botMessage: Message = { sender: 'bot', text: botText };
      setMessages((prevMessages) => [...prevMessages, botMessage]);

    } catch (error) {
      console.error('Failed to fetch from chatbot API:', error);
      const errorMessage: Message = { sender: 'bot', text: 'Sorry, I am having trouble connecting to my brain. Please try again later.' };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      <button className={styles.chatbotToggleButton} onClick={toggleChat}>
        {isOpen ? 'Close Chatbot' : 'Open Chatbot'}
      </button>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h2>Textbook Chatbot</h2>
            <button onClick={toggleChat} className={styles.closeButton}>×</button>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
                {/* ChatGPT-style markdown renderer */}
                {(() => {
                  // Split content into sections by the separator
                  const sections = msg.text.split(/\n\s*---\s*\n/);

                  return sections.map((section, sIdx) => {
                    // Split section into blocks (headings, paragraphs, lists)
                    const blocks = section.split(/\n\s*\n/).filter(block => block.trim() !== '');

                    return (
                      <div key={`section-${sIdx}`} className={styles.responseSection}>
                        {blocks.map((block, blockIdx) => {
                          // Check if this is a heading
                          if (block.startsWith('## ')) {
                            const headingText = block.substring(3).trim();
                            return (
                              <h3 key={`heading-${sIdx}-${blockIdx}`} className={styles.heading}>
                                {headingText.split(/(\[.*?\]\(.*?\))/g).map((part, idx) => {
                                  const linkMatch = /\[(.*?)\]\((.*?)\)/.exec(part);
                                  if (linkMatch) {
                                    return <a key={`hlink-${sIdx}-${blockIdx}-${idx}`} href={linkMatch[2]} target="_blank" rel="noopener noreferrer">{linkMatch[1]}</a>;
                                  }
                                  return part;
                                })}
                              </h3>
                            );
                          }
                          // Check if this is a subheading
                          else if (block.startsWith('### ')) {
                            const subheadingText = block.substring(4).trim();
                            return (
                              <h4 key={`subheading-${sIdx}-${blockIdx}`} className={styles.subheading}>
                                {subheadingText.split(/(\[.*?\]\(.*?\))/g).map((part, idx) => {
                                  const linkMatch = /\[(.*?)\]\((.*?)\)/.exec(part);
                                  if (linkMatch) {
                                    return <a key={`slink-${sIdx}-${blockIdx}-${idx}`} href={linkMatch[2]} target="_blank" rel="noopener noreferrer">{linkMatch[1]}</a>;
                                  }
                                  return part;
                                })}
                              </h4>
                            );
                          }
                          // Check if this is a source reference
                          else if (block.startsWith('**Source:**')) {
                            const sourceMatch = /\*\*Source:\*\* \[(.*?)\]\((.*?)\)/.exec(block);
                            if (sourceMatch) {
                              return (
                                <div key={`source-${sIdx}-${blockIdx}`} className={styles.sourceRef}>
                                  <strong>Source: </strong>
                                  <a href={sourceMatch[2]} target="_blank" rel="noopener noreferrer">{sourceMatch[1]}</a>
                                </div>
                              );
                            }
                          }
                          // Check if this is a list (starts with bullet points)
                          else if (block.split('\n').every(line => line.trim() === '' || line.trim().startsWith('- '))) {
                            const listItems = block.split('\n').filter(line => line.trim().startsWith('- '));
                            return (
                              <ul key={`list-${sIdx}-${blockIdx}`} className={styles.bulletList}>
                                {listItems.map((item, itemIdx) => {
                                  const itemText = item.substring(2); // Remove '- ' prefix
                                  return (
                                    <li key={`list-item-${sIdx}-${blockIdx}-${itemIdx}`} className={styles.listItem}>
                                      {itemText.split(/(\[.*?\]\(.*?\))/g).map((part, idx) => {
                                        const linkMatch = /\[(.*?)\]\((.*?)\)/.exec(part);
                                        if (linkMatch) {
                                          return <a key={`llink-${sIdx}-${blockIdx}-${itemIdx}-${idx}`} href={linkMatch[2]} target="_blank" rel="noopener noreferrer">{linkMatch[1]}</a>;
                                        }
                                        return part;
                                      })}
                                    </li>
                                  );
                                })}
                              </ul>
                            );
                          }
                          // Otherwise treat as a regular paragraph
                          else {
                            return (
                              <div key={`para-${sIdx}-${blockIdx}`} className={styles.paragraph}>
                                {block.split(/(\[.*?\]\(.*?\))/g).map((part, idx) => {
                                  const linkMatch = /\[(.*?)\]\((.*?)\)/.exec(part);
                                  if (linkMatch) {
                                    return <a key={`plink-${sIdx}-${blockIdx}-${idx}`} href={linkMatch[2]} target="_blank" rel="noopener noreferrer">{linkMatch[1]}</a>;
                                  }
                                  // Handle line breaks within the paragraph
                                  return part.split('\n').map((line, lineIdx) => (
                                    <React.Fragment key={`pline-${sIdx}-${blockIdx}-${idx}-${lineIdx}`}>
                                      {line}
                                      {lineIdx < part.split('\n').length - 1 && <br />}
                                    </React.Fragment>
                                  ));
                                })}
                              </div>
                            );
                          }
                        })}
                      </div>
                    );
                  });
                })()}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.bot}`}>
                <span className={styles.loadingDots}></span>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSubmit} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question..."
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
};

export default Chatbot;
