import React from 'react';
import Scene3D from './Scene3D';
import RobotModel from './RobotModel';

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
    <Scene3D
      cameraPosition={[8, 6, 8]}
      backgroundColor="#e0f7fa"
      showEnvironment={true}
      environmentPreset={environmentPreset}
      showShadows={showShadows}
    >
      {/* Digital twin environment elements */}
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
    </Scene3D>
  );
};

export default DigitalTwinScene;