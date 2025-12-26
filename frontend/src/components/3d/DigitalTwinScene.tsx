import React from 'react';
import { OrbitControls, PerspectiveCamera, Environment, Grid, Sky, ContactShadows } from '@react-three/drei';
import Scene3D from './Scene3D';
import RobotModel from './RobotModel';
import { Canvas } from '@react-three/fiber';

interface DigitalTwinSceneProps {
  robotPosition?: [number, number, number];
  environmentPreset?: string;
  showGrid?: boolean;
  showShadows?: boolean;
  robotModelPath?: string;
}

const DigitalTwinScene: React.FC<DigitalTwinSceneProps> = ({
  robotPosition = [0, 0, 0],
  environmentPreset = 'city',
  showGrid = true,
  showShadows = true,
  robotModelPath
}) => {
  return (
    <Scene3D cameraPosition={[8, 6, 8]} backgroundColor="#e0f7fa">
      {/* Digital twin environment elements */}
      <ambientLight intensity={0.5} />
      <directionalLight
        position={[10, 20, 15]}
        intensity={1}
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
      />

      {/* Ground plane with grid */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.5, 0]} receiveShadow>
        <planeGeometry args={[20, 20]} />
        <shadowMaterial opacity={0.2} />
      </mesh>

      {/* Robot model in the digital twin */}
      <RobotModel
        position={robotPosition}
        modelPath={robotModelPath}
        scale={1.5}
      />

      {/* Environment and effects */}
      <Environment preset={environmentPreset} />
      {showGrid && <Grid cellSize={1} cellThickness={0.5} cellColor="#6f6f6f" sectionSize={2} sectionThickness={1} sectionColor="#9d4b4b" fadeDistance={30} fadeStrength={1} position={[0, -0.5, 0]} />}
      {showShadows && <ContactShadows opacity={0.4} scale={10} blur={1} far={10} />}
      <Sky sunPosition={[100, 10, 100]} />

      <OrbitControls makeDefault enablePan={true} enableZoom={true} enableRotate={true} />
    </Scene3D>
  );
};

export default DigitalTwinScene;