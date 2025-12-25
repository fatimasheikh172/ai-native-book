import React from 'react';
import { ThemeProvider } from '../hooks/useTheme';
import ChatWidget from '../components/ChatWidget';

// This is the top-level customization root for Docusaurus
export default function Root({ children }) {
  return (
    <ThemeProvider>
      {children}
      <ChatWidget />
    </ThemeProvider>
  );
}