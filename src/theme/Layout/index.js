import React from 'react';
import Layout from '@theme-original/Layout';
import RAGChatbot from '@site/src/components/RAGChatbot'; // Adjust path if needed
import { ThemeProvider } from '@site/src/contexts/ThemeContext';
import { LanguageProvider } from '@site/src/contexts/LanguageContext';

export default function LayoutWrapper(props) {
  return (
    <ThemeProvider>
      <LanguageProvider>
        <Layout {...props} />
        <RAGChatbot />
      </LanguageProvider>
    </ThemeProvider>
  );
}
