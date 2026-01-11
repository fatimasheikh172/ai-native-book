import React from 'react';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';

const CustomMobileMenu = () => {
  const location = useLocation();

  const navItems = [
    {
      label: 'Modules',
      to: '#',
      dropdownItems: [
        { label: 'Module 1: Robotic Nervous System', to: '/docs/module-1-nervous-system/intro' },
        { label: 'Module 2: The Digital Twin', to: '/docs/module-2-digital-twin/intro' },
        { label: 'Module 3: The AI Robot Brain', to: '/docs/module-3-ai-brain/intro' },
        { label: 'Module 4: Vision-Language-Action (VLA)', to: '/docs/module-4-vla/index' },
      ]
    },
    { label: 'About', to: '/about' },
    { label: '3D Visualization', to: '/3d-explorer' },
    { label: 'GitHub', to: 'https://github.com/fatimasheikh172/ai-native-book', external: true }
  ];

  const isActive = (path) => {
    if (!path || typeof path !== 'string') return false;
    if (path.startsWith('http')) return false; // External links
    return location.pathname.startsWith(path);
  };

  return (
    <div className="custom-mobile-menu">
      <ul className="menu__list">
        {navItems.map((item, index) => (
          <li key={index} className="menu__list-item">
            {item.dropdownItems ? (
              <details className="dropdown__group">
                <summary className="dropdown__button menu__link">
                  <span className="dropdown__label">{item.label}</span>
                </summary>
                <ul className="dropdown__menu menu__list">
                  {item.dropdownItems.map((subItem, subIndex) => (
                    <li key={subIndex} className="dropdown__menu-item menu__list-item">
                      <Link
                        className={`menu__link ${isActive(subItem.to) ? 'menu__link--active' : ''}`}
                        to={subItem.to}
                        href={subItem.external ? subItem.to : undefined}
                        target={subItem.external ? "_blank" : undefined}
                        rel={subItem.external ? "noopener noreferrer" : undefined}
                      >
                        {subItem.label}
                      </Link>
                    </li>
                  ))}
                </ul>
              </details>
            ) : (
              <Link
                className={`menu__link ${isActive(item.to) ? 'menu__link--active' : ''}`}
                to={item.to}
                href={item.external ? item.to : undefined}
                target={item.external ? "_blank" : undefined}
                rel={item.external ? "noopener noreferrer" : undefined}
              >
                {item.label}
              </Link>
            )}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default CustomMobileMenu;