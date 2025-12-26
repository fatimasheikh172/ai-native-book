import React, { useState } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import Scene3D from '@site/src/components/3d/Scene3D';
import RotatingBox from '@site/src/components/3d/RotatingBox';
import RobotModel from '@site/src/components/3d/RobotModel';
import DigitalTwinScene from '@site/src/components/3d/DigitalTwinScene';
import styles from './3d-explorer.module.css';

function ThreeDExplorer() {
  const [activeScene, setActiveScene] = useState<'basic' | 'robot' | 'digitalTwin'>('digitalTwin');

  return (
    <Layout title="3D Explorer" description="Interactive 3D visualization for Physical AI and Robotics">
      <div className={clsx(styles.threeDExplorerContainer, 'hero hero--primary')} style={{
        background: 'radial-gradient(circle at 80% 20%, rgba(168, 85, 247, 0.15) 0%, transparent 40%), radial-gradient(circle at 20% 80%, rgba(0, 229, 255, 0.1) 0%, transparent 40%), #030712',
        padding: '4rem 0',
        minHeight: '100vh'
      }}>
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <div style={{ textAlign: 'center', marginBottom: '3rem' }}>
                <h1 style={{
                  fontSize: 'clamp(2.5rem, 5vw, 4.5rem)',
                  fontWeight: 900,
                  lineHeight: 1.1,
                  marginBottom: '1.5rem',
                  background: 'linear-gradient(to right, #fff 20%, #00e5ff 50%, #a855f7 80%)',
                  WebkitBackgroundClip: 'text',
                  backgroundClip: 'text',
                  WebkitTextFillColor: 'transparent',
                  filter: 'drop-shadow(0 0 15px rgba(0, 229, 255, 0.3))',
                  letterSpacing: '-1px'
                }}>
                  3D VISUALIZATION EXPLORER
                </h1>
                <p style={{
                  fontSize: '1.25rem',
                  color: '#94a3b8',
                  lineHeight: 1.6,
                  marginBottom: '2rem'
                }}>
                  Interactive 3D Environments for Physical AI & Robotics<br/>
                  <span style={{
                    display: 'block',
                    fontSize: '1rem',
                    fontWeight: 500,
                    color: '#a855f7',
                    textTransform: 'uppercase',
                    letterSpacing: '2px',
                    marginTop: '0.5rem'
                  }}>
                    Digital Twins • Robot Simulation • AI Integration
                  </span>
                </p>
                <div style={{
                  display: 'flex',
                  flexWrap: 'wrap',
                  justifyContent: 'center',
                  gap: '15px',
                  marginBottom: '2rem'
                }}>
                  <span style={{
                    background: 'rgba(255, 255, 255, 0.03)',
                    border: '1px solid rgba(255, 255, 255, 0.1)',
                    padding: '0.4rem 1rem',
                    borderRadius: '8px',
                    fontSize: '0.85rem',
                    color: '#f8fafc',
                    backdropFilter: 'blur(10px)',
                    transition: 'all 0.3s ease'
                  }}>
                    Three.js
                  </span>
                  <span style={{
                    background: 'rgba(255, 255, 255, 0.03)',
                    border: '1px solid rgba(255, 255, 255, 0.1)',
                    padding: '0.4rem 1rem',
                    borderRadius: '8px',
                    fontSize: '0.85rem',
                    color: '#f8fafc',
                    backdropFilter: 'blur(10px)',
                    transition: 'all 0.3s ease'
                  }}>
                    React Fiber
                  </span>
                  <span style={{
                    background: 'rgba(255, 255, 255, 0.03)',
                    border: '1px solid rgba(255, 255, 255, 0.1)',
                    padding: '0.4rem 1rem',
                    borderRadius: '8px',
                    fontSize: '0.85rem',
                    color: '#f8fafc',
                    backdropFilter: 'blur(10px)',
                    transition: 'all 0.3s ease'
                  }}>
                    Digital Twin
                  </span>
                  <span style={{
                    background: 'rgba(255, 255, 255, 0.03)',
                    border: '1px solid rgba(255, 255, 255, 0.1)',
                    padding: '0.4rem 1rem',
                    borderRadius: '8px',
                    fontSize: '0.85rem',
                    color: '#f8fafc',
                    backdropFilter: 'blur(10px)',
                    transition: 'all 0.3s ease'
                  }}>
                    Unity Simulation
                  </span>
                </div>
              </div>
            </div>
          </div>

          {/* Scene Selector */}
          <div className="row" style={{ marginBottom: '2rem' }}>
            <div className="col col--12">
              <div className={styles.sceneSelector}>
                <button
                  className={clsx(styles.sceneButton, {
                    [styles.active]: activeScene === 'basic'
                  })}
                  onClick={() => setActiveScene('basic')}
                  style={{
                    background: activeScene === 'basic'
                      ? 'linear-gradient(45deg, #00e5ff, #a855f7)'
                      : 'rgba(255, 255, 255, 0.03)',
                    border: activeScene === 'basic'
                      ? 'none'
                      : '1px solid rgba(255, 255, 255, 0.1)',
                    color: activeScene === 'basic' ? '#000' : '#f8fafc',
                    boxShadow: activeScene === 'basic' ? '0 0 15px rgba(0, 229, 255, 0.3)' : 'none'
                  }}
                >
                  Basic 3D Scene
                </button>
                <button
                  className={clsx(styles.sceneButton, {
                    [styles.active]: activeScene === 'robot'
                  })}
                  onClick={() => setActiveScene('robot')}
                  style={{
                    background: activeScene === 'robot'
                      ? 'linear-gradient(45deg, #00e5ff, #a855f7)'
                      : 'rgba(255, 255, 255, 0.03)',
                    border: activeScene === 'robot'
                      ? 'none'
                      : '1px solid rgba(255, 255, 255, 0.1)',
                    color: activeScene === 'robot' ? '#000' : '#f8fafc',
                    boxShadow: activeScene === 'robot' ? '0 0 15px rgba(0, 229, 255, 0.3)' : 'none'
                  }}
                >
                  Robot Model
                </button>
                <button
                  className={clsx(styles.sceneButton, {
                    [styles.active]: activeScene === 'digitalTwin'
                  })}
                  onClick={() => setActiveScene('digitalTwin')}
                  style={{
                    background: activeScene === 'digitalTwin'
                      ? 'linear-gradient(45deg, #00e5ff, #a855f7)'
                      : 'rgba(255, 255, 255, 0.03)',
                    border: activeScene === 'digitalTwin'
                      ? 'none'
                      : '1px solid rgba(255, 255, 255, 0.1)',
                    color: activeScene === 'digitalTwin' ? '#000' : '#f8fafc',
                    boxShadow: activeScene === 'digitalTwin' ? '0 0 15px rgba(0, 229, 255, 0.3)' : 'none'
                  }}
                >
                  Digital Twin
                </button>
              </div>
            </div>
          </div>

          {/* 3D Scene Display */}
          <div className="row">
            <div className="col col--12">
              <div className={styles.sceneDisplay}>
                <h3 className={styles.sceneTitle}>
                  {activeScene === 'basic' && 'Basic 3D Scene with Rotating Box'}
                  {activeScene === 'robot' && 'Robot Model Visualization'}
                  {activeScene === 'digitalTwin' && 'Digital Twin Environment'}
                </h3>

                <div style={{ flex: 1, minHeight: '400px' }}>
                  {activeScene === 'basic' && (
                    <Scene3D cameraPosition={[5, 5, 5]} backgroundColor="#030712">
                      <RotatingBox position={[0, 1, 0]} color="#00e5ff" />
                    </Scene3D>
                  )}
                  {activeScene === 'robot' && (
                    <Scene3D cameraPosition={[6, 4, 6]} backgroundColor="#030712">
                      <RobotModel position={[0, 0, 0]} />
                    </Scene3D>
                  )}
                  {activeScene === 'digitalTwin' && (
                    <DigitalTwinScene robotPosition={[0, 0, 0]} environmentPreset="city" />
                  )}
                </div>

                <p className={styles.sceneDescription}>
                  {activeScene === 'basic' && 'Interactive 3D scene with animated objects demonstrating core rendering capabilities.'}
                  {activeScene === 'robot' && 'Robot model visualization with detailed components and realistic rendering.'}
                  {activeScene === 'digitalTwin' && 'Complete digital twin environment with lighting, shadows, and physics simulation.'}
                </p>
              </div>
            </div>
          </div>

          {/* Key Concepts Section */}
          <div className="row" style={{ marginTop: '3rem' }}>
            <div className="col col--12">
              <h2 style={{
                textAlign: 'center',
                margin: '0 0 3rem 0',
                fontSize: 'clamp(2rem, 4vw, 3rem)',
                fontWeight: 800,
                color: '#ffffff',
                position: 'relative',
                textTransform: 'uppercase',
                letterSpacing: '2px'
              }}>
                3D Visualization Concepts
                <span style={{
                  content: '',
                  position: 'absolute',
                  bottom: '-15px',
                  left: '50%',
                  transform: 'translateX(-50%)',
                  width: '100px',
                  height: '4px',
                  background: 'linear-gradient(90deg, #00e5ff, #a855f7)',
                  borderRadius: '10px',
                  boxShadow: '0 0 15px #00e5ff'
                }}></span>
              </h2>
            </div>
          </div>

          <div className="row" style={{ marginTop: '2rem' }}>
            <div className="col col--4">
              <div style={{
                background: 'rgba(255, 255, 255, 0.03)',
                border: '1px solid rgba(255, 255, 255, 0.1)',
                backdropFilter: 'blur(12px)',
                borderRadius: '16px',
                padding: '2rem',
                transition: 'all 0.3s ease',
                height: '100%',
                display: 'flex',
                flexDirection: 'column'
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.transform = 'translateY(-5px)';
                e.currentTarget.style.borderColor = 'rgba(0, 229, 255, 0.3)';
                e.currentTarget.style.background = 'rgba(0, 229, 255, 0.05)';
                e.currentTarget.style.boxShadow = '0 10px 30px rgba(0, 0, 0, 0.3), 0 0 20px rgba(0, 229, 255, 0.1)';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.transform = 'translateY(0)';
                e.currentTarget.style.borderColor = 'rgba(255, 255, 255, 0.1)';
                e.currentTarget.style.background = 'rgba(255, 255, 255, 0.03)';
                e.currentTarget.style.boxShadow = 'none';
              }}
              >
                <div style={{ fontSize: '2.5rem', marginBottom: '1rem', textAlign: 'center' }}>🤖</div>
                <h3 style={{
                  color: '#00e5ff',
                  fontSize: '1.3rem',
                  fontWeight: 700,
                  margin: '0 0 1.5rem 0',
                  textAlign: 'center',
                  paddingBottom: '1rem',
                  borderBottom: '1px solid rgba(0, 229, 255, 0.2)'
                }}>
                  Robot Simulation
                </h3>
                <ul style={{ listStyle: 'none', padding: 0, margin: 0, flex: 1 }}>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Realistic robot model rendering
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Physics-based movement simulation
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Sensor integration visualization
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    ROS communication simulation
                  </li>
                </ul>
              </div>
            </div>

            <div className="col col--4">
              <div style={{
                background: 'rgba(255, 255, 255, 0.03)',
                border: '1px solid rgba(255, 255, 255, 0.1)',
                backdropFilter: 'blur(12px)',
                borderRadius: '16px',
                padding: '2rem',
                transition: 'all 0.3s ease',
                height: '100%',
                display: 'flex',
                flexDirection: 'column'
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.transform = 'translateY(-5px)';
                e.currentTarget.style.borderColor = 'rgba(0, 229, 255, 0.3)';
                e.currentTarget.style.background = 'rgba(0, 229, 255, 0.05)';
                e.currentTarget.style.boxShadow = '0 10px 30px rgba(0, 0, 0, 0.3), 0 0 20px rgba(0, 229, 255, 0.1)';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.transform = 'translateY(0)';
                e.currentTarget.style.borderColor = 'rgba(255, 255, 255, 0.1)';
                e.currentTarget.style.background = 'rgba(255, 255, 255, 0.03)';
                e.currentTarget.style.boxShadow = 'none';
              }}
              >
                <div style={{ fontSize: '2.5rem', marginBottom: '1rem', textAlign: 'center' }}>🌐</div>
                <h3 style={{
                  color: '#00e5ff',
                  fontSize: '1.3rem',
                  fontWeight: 700,
                  margin: '0 0 1.5rem 0',
                  textAlign: 'center',
                  paddingBottom: '1rem',
                  borderBottom: '1px solid rgba(0, 229, 255, 0.2)'
                }}>
                  Digital Twins
                </h3>
                <ul style={{ listStyle: 'none', padding: 0, margin: 0, flex: 1 }}>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Virtual replica of physical systems
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Real-time data visualization
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Environment simulation
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Unity integration capabilities
                  </li>
                </ul>
              </div>
            </div>

            <div className="col col--4">
              <div style={{
                background: 'rgba(255, 255, 255, 0.03)',
                border: '1px solid rgba(255, 255, 255, 0.1)',
                backdropFilter: 'blur(12px)',
                borderRadius: '16px',
                padding: '2rem',
                transition: 'all 0.3s ease',
                height: '100%',
                display: 'flex',
                flexDirection: 'column'
              }}
              onMouseEnter={(e) => {
                e.currentTarget.style.transform = 'translateY(-5px)';
                e.currentTarget.style.borderColor = 'rgba(0, 229, 255, 0.3)';
                e.currentTarget.style.background = 'rgba(0, 229, 255, 0.05)';
                e.currentTarget.style.boxShadow = '0 10px 30px rgba(0, 0, 0, 0.3), 0 0 20px rgba(0, 229, 255, 0.1)';
              }}
              onMouseLeave={(e) => {
                e.currentTarget.style.transform = 'translateY(0)';
                e.currentTarget.style.borderColor = 'rgba(255, 255, 255, 0.1)';
                e.currentTarget.style.background = 'rgba(255, 255, 255, 0.03)';
                e.currentTarget.style.boxShadow = 'none';
              }}
              >
                <div style={{ fontSize: '2.5rem', marginBottom: '1rem', textAlign: 'center' }}>🧠</div>
                <h3 style={{
                  color: '#00e5ff',
                  fontSize: '1.3rem',
                  fontWeight: 700,
                  margin: '0 0 1.5rem 0',
                  textAlign: 'center',
                  paddingBottom: '1rem',
                  borderBottom: '1px solid rgba(0, 229, 255, 0.2)'
                }}>
                  AI Integration
                </h3>
                <ul style={{ listStyle: 'none', padding: 0, margin: 0, flex: 1 }}>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    AI decision-making visualization
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Path planning algorithms
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Behavior tree rendering
                  </li>
                  <li style={{ color: '#cbd5e1', fontSize: '0.9rem', lineHeight: 1.6, marginBottom: '0.8rem', position: 'relative', paddingLeft: '1.2rem' }}>
                    <span style={{ color: '#a855f7', position: 'absolute', left: 0, top: '0.2rem' }}>•</span>
                    Perception-action loops
                  </li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ThreeDExplorer;