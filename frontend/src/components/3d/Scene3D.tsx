import React, { Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Environment, Grid } from '@react-three/drei';
import * as THREE from 'three';

interface Scene3DProps {
  children?: React.ReactNode;
  cameraPosition?: [number, number, number];
  showGrid?: boolean;
  showEnvironment?: boolean;
  backgroundColor?: string;
}

const Scene3D: React.FC<Scene3DProps> = ({
  children,
  cameraPosition = [5, 5, 5],
  showGrid = true,
  showEnvironment = true,
  backgroundColor = '#f0f0f0'
}) => {
  return (
    <div style={{ width: '100%', height: '500px', position: 'relative' }}>
      <Canvas
        camera={{ position: cameraPosition, fov: 50 }}
        style={{ background: backgroundColor }}
      >
        <Suspense fallback={null}>
          <ambientLight intensity={0.5} />
          <pointLight position={[10, 10, 10]} intensity={1} />
          <pointLight position={[-10, -10, -10]} intensity={0.5} />

          {children}

          {showGrid && <Grid cellSize={1} cellThickness={0.5} cellColor="#6f6f6f" sectionSize={2} sectionThickness={1} sectionColor="#9d4b4b" fadeDistance={30} fadeStrength={1} />}
          {showEnvironment && <Environment preset="city" />}
          <OrbitControls makeDefault enablePan={true} enableZoom={true} enableRotate={true} />
        </Suspense>
      </Canvas>
    </div>
  );
};

export default Scene3D;