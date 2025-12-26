import React, { Suspense } from 'react';
import { useGLTF } from '@react-three/drei';
import { MeshProps } from '@react-three/fiber';

interface RobotModelProps extends MeshProps {
  modelPath?: string;
  scale?: number | [number, number, number];
  position?: [number, number, number];
  rotation?: [number, number, number];
}

// Default robot model (simple representation)
const DefaultRobotModel: React.FC<MeshProps> = (props) => {
  return (
    <group {...props}>
      {/* Robot body */}
      <mesh position={[0, 1, 0]}>
        <boxGeometry args={[1.5, 1.5, 1]} />
        <meshStandardMaterial color="#4a90e2" />
      </mesh>

      {/* Robot head */}
      <mesh position={[0, 2.2, 0]}>
        <boxGeometry args={[0.8, 0.8, 0.8]} />
        <meshStandardMaterial color="#7ed321" />
      </mesh>

      {/* Robot left arm */}
      <mesh position={[-1.2, 1, 0]} rotation={[0, 0, Math.PI / 4]}>
        <cylinderGeometry args={[0.15, 0.15, 1.2]} />
        <meshStandardMaterial color="#f5a623" />
      </mesh>

      {/* Robot right arm */}
      <mesh position={[1.2, 1, 0]} rotation={[0, 0, -Math.PI / 4]}>
        <cylinderGeometry args={[0.15, 0.15, 1.2]} />
        <meshStandardMaterial color="#f5a623" />
      </mesh>

      {/* Robot left leg */}
      <mesh position={[-0.4, -0.8, 0]}>
        <cylinderGeometry args={[0.2, 0.2, 1.5]} />
        <meshStandardMaterial color="#d0011b" />
      </mesh>

      {/* Robot right leg */}
      <mesh position={[0.4, -0.8, 0]}>
        <cylinderGeometry args={[0.2, 0.2, 1.5]} />
        <meshStandardMaterial color="#d0011b" />
      </mesh>
    </group>
  );
};

const RobotModel: React.FC<RobotModelProps> = ({
  modelPath,
  scale = 1,
  position = [0, 0, 0],
  rotation = [0, 0, 0],
  ...props
}) => {
  // If no model path is provided, use the default robot model
  if (!modelPath) {
    return (
      <DefaultRobotModel
        scale={scale}
        position={position}
        rotation={rotation}
        {...props}
      />
    );
  }

  // Component to load external GLTF model
  const GLTFModel = () => {
    const { scene } = useGLTF(modelPath);
    return <primitive object={scene} scale={scale} position={position} rotation={rotation} {...props} />;
  };

  return (
    <Suspense fallback={null}>
      <GLTFModel />
    </Suspense>
  );
};

export default RobotModel;