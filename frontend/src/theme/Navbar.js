import React from 'react';
import Navbar from '@theme-original/Navbar';
import { useTheme } from '../hooks/useTheme';
import clsx from 'clsx';

import styles from './Navbar/index.module.css';

function ThemeToggle() {
  const { theme, toggleTheme } = useTheme();

  return (
    <button
      className={clsx('button button--sm', styles.themeToggleButton)}
      onClick={toggleTheme}
      aria-label={`Switch to ${theme === 'light' ? 'dark' : 'light'} mode`}
      title={`Switch to ${theme === 'light' ? 'dark' : 'light'} mode`}
    >
      {theme === 'light' ? '🌙' : '☀️'}
    </button>
  );
}

export default function NavbarWrapper(props) {
  return (
    <>
      <Navbar {...props} />
      <div className={styles.themeToggleContainer}>
        <ThemeToggle />
      </div>
    </>
  );
}