---
sidebar_position: 5
---

# Module 1 Technical Diagrams: ROS Architecture

## Overview

This section provides technical diagrams illustrating the architecture and concepts of the Robotic Nervous System, focusing on ROS 2 architecture, NORD framework, and system integration patterns. These diagrams visualize the key concepts covered in Module 1.

## ROS 2 Architecture Diagrams

### Client Library Architecture

```mermaid
graph TB
    subgraph "Application Layer"
        A1["Robot Application"]
        A2["Sensor Processing"]
        A3["Control Algorithms"]
    end

    subgraph "ROS 2 Client Libraries"
        CL1["rclcpp (C++)"]
        CL2["rclpy (Python)"]
        CL3["rcl (Base C Library)"]
    end

    subgraph "ROS Middleware (rmw)"
        RMW1["ROS Middleware Abstraction"]
    end

    subgraph "DDS Implementation"
        DDS1["DDS Provider A"]
        DDS2["DDS Provider B"]
        DDS3["DDS Provider C"]
    end

    A1 --> CL1
    A2 --> CL2
    A3 --> CL1
    CL1 --> RMW1
    CL2 --> RMW1
    CL3 --> RMW1
    RMW1 --> DDS1
    RMW1 --> DDS2
    RMW1 --> DDS3

    style A1 fill:#e3f2fd
    style A2 fill:#e3f2fd
    style A3 fill:#e3f2fd
    style CL1 fill:#f3e5f5
    style CL2 fill:#f3e5f5
    style CL3 fill:#f3e5f5
    style RMW1 fill:#e8f5e8
    style DDS1 fill:#fff3e0
    style DDS2 fill:#fff3e0
    style DDS3 fill:#fff3e0
```

### Node Communication Pattern

```mermaid
graph LR
    subgraph "ROS 2 Network"
        N1["Node A"]
        N2["Node B"]
        N3["Node C"]
        LC["Launch Controller"]
    end

    subgraph "Topics"
        T1["/sensor_data"]
        T2["/cmd_vel"]
        T3["/robot_state"]
    end

    subgraph "Services"
        S1["/get_map"]
        S2["/set_mode"]
    end

    subgraph "Actions"
        A1["/move_base"]
        A2["/follow_path"]
    end

    N1 --> T1
    T1 --> N2
    N2 --> T2
    T2 --> N3
    N3 --> T3
    T3 --> N1

    N1 <--> S1
    N2 <--> S2

    N1 <--> A1
    N2 <--> A2

    LC -.-> N1
    LC -.-> N2
    LC -.-> N3

    style N1 fill:#e3f2fd
    style N2 fill:#e3f2fd
    style N3 fill:#e3f2fd
    style T1 fill:#fff9c4
    style T2 fill:#fff9c4
    style T3 fill:#fff9c4
    style S1 fill:#ffccbc
    style S2 fill:#ffccbc
    style A1 fill:#d1c4e9
    style A2 fill:#d1c4e9
    style LC fill:#b2ebf2
```

### Quality of Service Configuration

```mermaid
graph TD
    subgraph "Publisher Configuration"
        P["Publisher"]
        R1["Reliability: RELIABLE"]
        D1["Durability: TRANSIENT_LOCAL"]
        H1["History: KEEP_LAST"]
        L1["Depth: 10"]
    end

    subgraph "Subscriber Configuration"
        S["Subscriber"]
        R2["Reliability: RELIABLE"]
        D2["Durability: VOLATILE"]
        H2["History: KEEP_ALL"]
        L2["Depth: UNLIMITED"]
    end

    subgraph "Topic Connection"
        T["Topic: /sensor_data"]
    end

    P --> R1
    P --> D1
    P --> H1
    P --> L1
    S --> R2
    S --> D2
    S --> H2
    S --> L2

    R1 <--> T
    D1 <--> T
    H1 <--> T
    L1 <--> T
    R2 <--> T
    D2 <--> T
    H2 <--> T
    L2 <--> T

    style P fill:#e3f2fd
    style S fill:#e3f2fd
    style T fill:#f3e5f5
    style R1 fill:#e8f5e8
    style D1 fill:#e8f5e8
    style H1 fill:#e8f5e8
    style L1 fill:#e8f5e8
    style R2 fill:#e8f5e8
    style D2 fill:#e8f5e8
    style H2 fill:#e8f5e8
    style L2 fill:#e8f5e8
```

## NORD Framework Architecture

### NORD System Components

```mermaid
graph TB
    subgraph "Application Layer"
        RL["Robot Logic"]
        SL["Sensor Logic"]
        CL["Control Logic"]
    end

    subgraph "NORD Framework"
        RD["Robot Description"]
        PM["Physics Manager"]
        MS["Material System"]
        SS["Sensor System"]
    end

    subgraph "Omniverse Integration"
        USD["USD Export/Import"]
        RT["Ray Tracing"]
        PS["Physics Simulation"]
        AN["Animation System"]
    end

    subgraph "Simulation Layer"
        ENV["Environment Model"]
        OBJ["Object Models"]
        SENS["Sensor Models"]
    end

    RL --> RD
    SL --> SS
    CL --> PM

    RD --> USD
    PM --> PS
    MS --> RT
    SS --> SENS

    USD --> ENV
    PS --> OBJ
    RT --> SENS

    style RL fill:#e3f2fd
    style SL fill:#e3f2fd
    style CL fill:#e3f2fd
    style RD fill:#f3e5f5
    style PM fill:#f3e5f5
    style MS fill:#f3e5f5
    style SS fill:#f3e5f5
    style USD fill:#e8f5e8
    style RT fill:#e8f5e8
    style PS fill:#e8f5e8
    style AN fill:#e8f5e8
    style ENV fill:#fff3e0
    style OBJ fill:#fff3e0
    style SENS fill:#fff3e0
```

### NORD Replay System Architecture

```mermaid
graph LR
    subgraph "Recording Components"
        RE["Recording Engine"]
        DM["Data Manager"]
        BF["Buffer System"]
        CF["Compression"]
    end

    subgraph "Storage Layer"
        HDF["HDF5 Storage"]
        IDX["Index System"]
        MET["Metadata Store"]
    end

    subgraph "Playback Components"
        PE["Playback Engine"]
        IN["Interpolator"]
        VT["Visualization"]
        AE["Analysis Engine"]
    end

    subgraph "Data Flow"
        JS["Joint States"]
        SD["Sensor Data"]
        ES["Env State"]
        CC["Commands"]
    end

    RE --> DM
    DM --> BF
    BF --> CF
    CF --> HDF
    HDF --> IDX
    HDF --> MET

    HDF --> PE
    IDX --> PE
    MET --> PE
    PE --> IN
    IN --> VT
    IN --> AE

    RE --> JS
    RE --> SD
    RE --> ES
    RE --> CC

    JS --> DM
    SD --> DM
    ES --> DM
    CC --> DM

    style RE fill:#e3f2fd
    style DM fill:#e3f2fd
    style BF fill:#e3f2fd
    style CF fill:#e3f2fd
    style HDF fill:#f3e5f5
    style IDX fill:#f3e5f5
    style MET fill:#f3e5f5
    style PE fill:#e8f5e8
    style IN fill:#e8f5e8
    style VT fill:#e8f5e8
    style AE fill:#e8f5e8
    style JS fill:#fff9c4
    style SD fill:#fff9c4
    style ES fill:#fff9c4
    style CC fill:#fff9c4
```

## URDF and Robot Modeling Diagrams

### URDF Structure Hierarchy

```mermaid
graph TD
    ROBOT["Robot Definition"] --> MATERIALS["Materials"]
    ROBOT --> LINKS["Links"]
    ROBOT --> JOINTS["Joints"]
    ROBOT --> GAZBO["Gazebo Plugins"]

    LINKS --> L1["Link 1"]
    LINKS --> L2["Link 2"]
    LINKS --> L3["Link N"]

    JOINTS --> J1["Joint 1"]
    JOINTS --> J2["Joint 2"]
    JOINTS --> J3["Joint N"]

    L1 --> VISUAL["Visual Mesh"]
    L1 --> COLLISION["Collision Mesh"]
    L1 --> INERTIAL["Inertial Properties"]

    L2 --> VISUAL2["Visual Mesh"]
    L2 --> COLLISION2["Collision Mesh"]
    L2 --> INERTIAL2["Inertial Properties"]

    J1 --> PARENT["Parent Link"]
    J1 --> CHILD["Child Link"]
    J1 --> LIMITS["Joint Limits"]
    J1 --> ORIGIN["Origin Transform"]

    style ROBOT fill:#e3f2fd
    style MATERIALS fill:#f3e5f5
    style LINKS fill:#f3e5f5
    style JOINTS fill:#f3e5f5
    style GAZBO fill:#f3e5f5
    style L1 fill:#e8f5e8
    style L2 fill:#e8f5e8
    style L3 fill:#e8f5e8
    style J1 fill:#fff3e0
    style J2 fill:#fff3e0
    style J3 fill:#fff3e0
    style VISUAL fill:#fff9c4
    style COLLISION fill:#fff9c4
    style INERTIAL fill:#fff9c4
    style VISUAL2 fill:#fff9c4
    style COLLISION2 fill:#fff9c4
    style INERTIAL2 fill:#fff9c4
    style PARENT fill:#ffccbc
    style CHILD fill:#ffccbc
    style LIMITS fill:#ffccbc
    style ORIGIN fill:#ffccbc
```

### Robot Kinematic Chain

```mermaid
graph LR
    BASE["Base Link"] --> J1["Joint 1"]
    J1 --> L1["Link 1"]
    L1 --> J2["Joint 2"]
    J2 --> L2["Link 2"]
    L2 --> J3["Joint 3"]
    J3 --> L3["Link 3"]
    L3 --> J4["Joint 4"]
    J4 --> EE["End Effector"]

    subgraph "Kinematic Properties"
        KP1["Joint Type: Revolute"]
        KP2["Joint Limits"]
        KP3["DH Parameters"]
        KP4["Forward Kinematics"]
        KP5["Inverse Kinematics"]
    end

    J1 -.-> KP1
    J2 -.-> KP2
    L1 -.-> KP3
    L2 -.-> KP4
    L3 -.-> KP5

    style BASE fill:#e3f2fd
    style J1 fill:#f3e5f5
    style L1 fill:#e8f5e8
    style J2 fill:#f3e5f5
    style L2 fill:#e8f5e8
    style J3 fill:#f3e5f5
    style L3 fill:#e8f5e8
    style J4 fill:#f3e5f5
    style EE fill:#ffcdd2
    style KP1 fill:#fff9c4
    style KP2 fill:#fff9c4
    style KP3 fill:#fff9c4
    style KP4 fill:#fff9c4
    style KP5 fill:#fff9c4
```

## System Integration Patterns

### Sim-to-Real Transfer Architecture

```mermaid
graph LR
    subgraph "Physical Robot"
        PR["Real Robot Hardware"]
        PS["Real Sensors"]
        PA["Real Actuators"]
        PC["Real Controller"]
    end

    subgraph "Simulation Environment"
        SR["Simulated Robot"]
        SS["Simulated Sensors"]
        SA["Simulated Actuators"]
        SC["Simulated Controller"]
    end

    subgraph "Transfer Layer"
        RL["Reinforcement Learning"]
        IL["Imitation Learning"]
        DA["Domain Adaptation"]
    end

    subgraph "AI Algorithms"
        NN["Neural Networks"]
        RL2["Reinforcement Learning"]
        MPC["Model Predictive Control"]
    end

    PS --> RL
    SS --> RL
    PA --> IL
    SA --> IL
    DA <--> SR
    DA <--> PR
    RL2 --> DA
    RL2 --> MPC

    style PR fill:#ffcdd2
    style PS fill:#ffcdd2
    style PA fill:#ffcdd2
    style PC fill:#ffcdd2
    style SR fill:#e8f5e8
    style SS fill:#e8f5e8
    style SA fill:#e8f5e8
    style SC fill:#e8f5e8
    style RL fill:#fff9c4
    style IL fill:#fff9c4
    style DA fill:#fff9c4
    style NN fill:#e3f2fd
    style RL2 fill:#e3f2fd
    style MPC fill:#e3f2fd
```

### Middleware Communication Architecture

```mermaid
graph TB
    subgraph "Robot Hardware Layer"
        MC["Microcontroller"]
        S1["IMU Sensor"]
        S2["Camera"]
        S3["LIDAR"]
        M1["Motor Driver"]
        M2["Gripper"]
    end

    subgraph "ROS 2 Nodes"
        ND["Driver Node"]
        NS1["Sensor Node 1"]
        NS2["Sensor Node 2"]
        NS3["Sensor Node 3"]
        NC["Control Node"]
        NP["Perception Node"]
        NB["Behavior Node"]
    end

    subgraph "Communication Layer"
        T1["/imu/data"]
        T2["/camera/image"]
        T3["/lidar/scan"]
        T4["/cmd_vel"]
        T5["/joint_states"]
        T6["/perception/objects"]
        T7["/behavior/cmd"]
    end

    subgraph "External Systems"
        RV["RViz Visualizer"]
        ROS["ROS Bridge"]
        WEB["Web Interface"]
    end

    MC <--> ND
    S1 <--> NS1
    S2 <--> NS2
    S3 <--> NS3
    M1 <--> NC
    M2 <--> NC

    ND <--> T5
    NS1 <--> T1
    NS2 <--> T2
    NS3 <--> T3
    NC <--> T4
    NP <--> T6
    NB <--> T7

    T1 <--> NP
    T2 <--> NP
    T3 <--> NP
    T6 <--> NB
    T7 <--> NC

    RV <--> T1
    RV <--> T2
    RV <--> T3
    ROS <--> T4
    WEB <--> T7

    style MC fill:#ffcdd2
    style S1 fill:#ffcdd2
    style S2 fill:#ffcdd2
    style S3 fill:#ffcdd2
    style M1 fill:#ffcdd2
    style M2 fill:#ffcdd2
    style ND fill:#e3f2fd
    style NS1 fill:#e3f2fd
    style NS2 fill:#e3f2fd
    style NS3 fill:#e3f2fd
    style NC fill:#e3f2fd
    style NP fill:#e3f2fd
    style NB fill:#e3f2fd
    style T1 fill:#f3e5f5
    style T2 fill:#f3e5f5
    style T3 fill:#f3e5f5
    style T4 fill:#f3e5f5
    style T5 fill:#f3e5f5
    style T6 fill:#f3e5f5
    style T7 fill:#f3e5f5
    style RV fill:#e8f5e8
    style ROS fill:#e8f5e8
    style WEB fill:#e8f5e8
```

These technical diagrams illustrate the core concepts of the Robotic Nervous System, including ROS 2 architecture, NORD framework integration, URDF modeling, and system integration patterns that form the foundation of Physical AI systems.