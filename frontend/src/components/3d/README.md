# 3D Visualization Components

This directory contains the 3D visualization components for the Physical AI and Human-Aided Robotics book project.

## Components

### Scene3D
The core 3D scene component that wraps the Three.js canvas with default lighting, camera, and controls.

### RotatingBox
A simple animated 3D object to demonstrate basic animation capabilities.

### RobotModel
A component that can display either a default robot model or load external GLTF models.

### DigitalTwinScene
A complete digital twin environment with robot, lighting, shadows, and environment effects.

## Usage

To use these components in Docusaurus pages:

```tsx
import Scene3D from '@site/src/components/3d/Scene3D';
import RobotModel from '@site/src/components/3d/RobotModel';

function MyPage() {
  return (
    <Scene3D>
      <RobotModel position={[0, 0, 0]} />
    </Scene3D>
  );
}
```

## Features

- Interactive 3D scenes with orbit controls
- GLTF model loading support
- Environment maps and lighting
- Shadow effects
- Responsive design
- Performance optimized rendering