// Theme toggle functionality
document.addEventListener('DOMContentLoaded', function() {
  const themeToggleContainer = document.getElementById('theme-toggle-container');

  if (themeToggleContainer) {
    // Check for saved theme preference or system preference
    const savedTheme = localStorage.getItem('theme');
    const systemPrefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;

    let currentTheme = 'light';
    if (savedTheme) {
      currentTheme = savedTheme;
    } else if (systemPrefersDark) {
      currentTheme = 'dark';
    }

    // Apply the initial theme
    document.documentElement.setAttribute('data-theme', currentTheme);

    // Create the theme toggle button
    const themeToggle = document.createElement('button');
    themeToggle.className = 'theme-toggle-button';
    themeToggle.setAttribute('aria-label', `Switch to ${currentTheme === 'light' ? 'dark' : 'light'} mode`);
    themeToggle.title = `Switch to ${currentTheme === 'light' ? 'dark' : 'light'} mode`;

    // Set the initial icon
    themeToggle.innerHTML = currentTheme === 'light' ? '🌙' : '☀️';

    // Add click event to toggle theme
    themeToggle.addEventListener('click', function() {
      const newTheme = document.documentElement.getAttribute('data-theme') === 'light' ? 'dark' : 'light';

      // Update the theme
      document.documentElement.setAttribute('data-theme', newTheme);

      // Update the button icon
      themeToggle.innerHTML = newTheme === 'light' ? '🌙' : '☀️';
      themeToggle.setAttribute('aria-label', `Switch to ${newTheme === 'light' ? 'dark' : 'light'} mode`);
      themeToggle.title = `Switch to ${newTheme === 'light' ? 'dark' : 'light'} mode`;

      // Save the theme preference
      localStorage.setItem('theme', newTheme);

      // Dispatch a custom event for other components to listen to
      window.dispatchEvent(new CustomEvent('themeChange', { detail: { theme: newTheme } }));
    });

    themeToggleContainer.appendChild(themeToggle);
  }
});