import React from 'react';
import { useFrame } from '@react-three/fiber';
import { MeshProps } from '@react-three/fiber';
import * as THREE from 'three';

interface RotatingBoxProps extends MeshProps {
  speed?: number;
  color?: string;
}

const RotatingBox: React.FC<RotatingBoxProps> = ({ speed = 1, color = '#61dafb', ...props }) => {
  const meshRef = React.useRef<THREE.Mesh>(null!);

  useFrame(() => {
    meshRef.current.rotation.x += 0.01 * speed;
    meshRef.current.rotation.y += 0.01 * speed;
  });

  return (
    <mesh {...props} ref={meshRef}>
      <boxGeometry args={[2, 2, 2]} />
      <meshStandardMaterial color={color} />
    </mesh>
  );
};

export default RotatingBox;