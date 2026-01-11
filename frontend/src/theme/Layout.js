import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { useLocation } from '@docusaurus/router';

export default function Layout(props) {
  const location = useLocation();

  // For mobile navigation, we rely on Docusaurus's built-in sidebar
  // The navbar configuration in docusaurus.config.js determines what appears in mobile

  return (
    <OriginalLayout {...props} />
  );
}