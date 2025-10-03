# Bevy Regolith - Architecture Diagrams

## System Flow Diagram

```mermaid
graph TD
    A[User Input] --> B[Spawn Particles]
    B --> C[Particle Entities]
    
    C --> D[FixedUpdate Loop]
    
    D --> E[1. Predict Positions]
    E --> F[Apply Gravity]
    F --> G[Integrate Velocity]
    
    G --> H[2. Rebuild Spatial Hash]
    H --> I[Grid Cell Assignment]
    
    I --> J[3. Solve Constraints]
    J --> K[Particle-Particle Collision]
    J --> L[Ground Plane Collision]
    
    K --> M[4. Update Velocities]
    L --> M
    M --> N[Position Delta / dt]
    
    N --> O[5. Apply Damping]
    
    O --> P[Update Rendering]
    P --> Q[Sync Transforms]
    
    Q --> R[Debug Visualization]
    R --> S[Display Frame]
```

## Component Architecture

```mermaid
graph LR
    subgraph "Particle Entity"
        A[ParticlePosition]
        B[ParticleVelocity]
        C[ParticlePrevPosition]
        D[ParticleMass]
        E[ParticleMaterial]
        F[ParticleRadius]
        G[ActiveParticle]
        H[Transform]
        I[PbrBundle]
    end
    
    subgraph "Resources"
        J[SolverConfig]
        K[MaterialRegistry]
        L[SpatialHash]
    end
    
    A --> L
    E --> K
    J --> A
    J --> B
```

## PBD Solver Pipeline

```mermaid
sequenceDiagram
    participant Input
    participant Particles
    participant SpatialHash
    participant Solver
    participant Renderer
    
    Input->>Particles: Spawn new particles
    
    loop Every FixedUpdate
        Solver->>Particles: Read positions & velocities
        Solver->>Particles: Apply gravity
        Solver->>Particles: Predict new positions
        
        Solver->>SpatialHash: Rebuild grid
        SpatialHash-->>Solver: Grid ready
        
        loop Constraint Iterations
            Solver->>SpatialHash: Query neighbors
            SpatialHash-->>Solver: Neighbor list
            Solver->>Particles: Solve collisions
            Solver->>Particles: Apply corrections
        end
        
        Solver->>Particles: Update velocities
        Solver->>Particles: Apply damping
    end
    
    loop Every Update
        Particles->>Renderer: Sync transforms
        Renderer->>Screen: Draw particles
    end
```

## Spatial Hash Structure

```mermaid
graph TD
    subgraph "World Space"
        A[Particle at 1.5, 2.3, 0.8]
    end
    
    A --> B[world_to_cell]
    B --> C[Cell 15, 23, 8]
    
    C --> D[HashMap]
    
    subgraph "Spatial Hash Grid"
        D --> E[Cell 15,23,8: Entity1, Entity5]
        D --> F[Cell 15,23,9: Entity2]
        D --> G[Cell 16,23,8: Entity3, Entity4]
    end
    
    H[Query Neighbors] --> I[Get cell + neighbors]
    I --> D
    D --> J[Return: Entity1, Entity2, Entity3, Entity4, Entity5]
```

## Material System

```mermaid
classDiagram
    class Material {
        +f32 density
        +f32 friction
        +f32 restitution
        +f32 particle_radius
        +Color color
        +lunar_regolith() Material
        +sand() Material
        +snow() Material
    }
    
    class MaterialRegistry {
        +Vec~Material~ materials
        +add(Material) usize
        +get(usize) Material
    }
    
    class ParticleMaterial {
        +usize material_id
    }
    
    MaterialRegistry "1" --> "*" Material
    ParticleMaterial --> MaterialRegistry
```

## Collision Detection Flow

```mermaid
flowchart TD
    A[Start Constraint Iteration] --> B{For each particle}
    
    B --> C[Query spatial hash]
    C --> D[Get neighbors within radius]
    
    D --> E{For each neighbor}
    E --> F[Calculate distance]
    F --> G{Distance < min_dist?}
    
    G -->|Yes| H[Calculate penetration]
    H --> I[Compute correction vector]
    I --> J[Apply mass-weighted correction]
    
    G -->|No| E
    J --> E
    
    E -->|Done| K[Check ground collision]
    K --> L{Y < radius?}
    
    L -->|Yes| M[Set Y = radius]
    L -->|No| N[Next particle]
    
    M --> N
    N --> B
    
    B -->|Done| O{More iterations?}
    O -->|Yes| A
    O -->|No| P[End]
```

## Data Flow: CPU to GPU Migration

```mermaid
graph TB
    subgraph "Phase 1: CPU Prototype"
        A1[Bevy ECS Components]
        A2[CPU Solver Systems]
        A3[Direct Memory Access]
    end
    
    subgraph "Phase 2: GPU Compute"
        B1[Bevy ECS Components]
        B2[GPU Buffers]
        B3[Compute Shaders]
        B4[CPU-GPU Sync]
    end
    
    subgraph "Phase 3: Regolith Integration"
        C1[Bevy ECS Wrapper]
        C2[Regolith Engine]
        C3[GPU Compute]
        C4[Minimal Sync]
    end
    
    A1 --> A2
    A2 --> A3
    
    B1 --> B4
    B4 --> B2
    B2 --> B3
    B3 --> B2
    B2 --> B4
    B4 --> B1
    
    C1 --> C2
    C2 --> C3
    C3 --> C2
    C2 --> C1
    
    A1 -.Migration.-> B1
    B1 -.Integration.-> C1
```

## Performance Comparison

```mermaid
graph LR
    subgraph "CPU Implementation"
        A1[5-10k particles]
        A2[~5ms update]
        A3[Easy debugging]
    end
    
    subgraph "GPU Compute"
        B1[100-500k particles]
        B2[~5ms update]
        B3[Harder debugging]
    end
    
    subgraph "Regolith Engine"
        C1[500k-1M particles]
        C2[~5-8ms update]
        C3[Production ready]
    end
    
    A1 -.Optimize.-> B1
    B1 -.Integrate.-> C1
```

## Rigid Body Interaction

```mermaid
sequenceDiagram
    participant RigidBody
    participant Particles
    participant SpatialHash
    participant PhysicsEngine
    
    Note over RigidBody,PhysicsEngine: Each Physics Step
    
    RigidBody->>SpatialHash: Query nearby particles
    SpatialHash-->>RigidBody: Particle list
    
    loop For each nearby particle
        RigidBody->>Particles: Check collision
        Particles-->>RigidBody: Contact info
        
        alt Collision detected
            RigidBody->>RigidBody: Accumulate force
            RigidBody->>RigidBody: Accumulate torque
            RigidBody->>Particles: Apply reaction force
        end
    end
    
    RigidBody->>PhysicsEngine: Apply accumulated forces
    PhysicsEngine->>RigidBody: Update transform
```

## Camera System

```mermaid
stateDiagram-v2
    [*] --> Idle
    
    Idle --> Rotating: Right Mouse Down
    Rotating --> Idle: Right Mouse Up
    
    Idle --> Zooming: Mouse Wheel
    Zooming --> Idle: Wheel Stop
    
    Idle --> Panning: WASD/QE Keys
    Panning --> Idle: Keys Released
    
    Rotating --> Rotating: Mouse Motion
    Panning --> Panning: Key Held
    
    note right of Rotating
        Update yaw & pitch
        Clamp pitch
    end note
    
    note right of Zooming
        Adjust distance
        Clamp range
    end note
    
    note right of Panning
        Move focus point
        In camera space
    end note
```

## Debug Visualization Layers

```mermaid
graph TD
    A[Debug System] --> B{Enabled Features}
    
    B --> C[Spatial Grid]
    C --> C1[Draw occupied cells]
    C --> C2[Color by density]
    
    B --> D[Velocities]
    D --> D1[Draw velocity vectors]
    D --> D2[Scale by magnitude]
    
    B --> E[Constraints]
    E --> E1[Draw collision pairs]
    E --> E2[Show penetration]
    
    B --> F[Forces]
    F --> F1[Draw force vectors]
    F --> F2[Show rigid body contacts]
```

## Module Dependencies

```mermaid
graph TD
    A[lib.rs] --> B[particle.rs]
    A --> C[material.rs]
    A --> D[spawner.rs]
    A --> E[solver.rs]
    A --> F[spatial.rs]
    A --> G[collision.rs]
    A --> H[rendering.rs]
    A --> I[camera.rs]
    A --> J[ui.rs]
    A --> K[debug.rs]
    A --> L[rigid_body.rs]
    
    D --> B
    D --> C
    
    E --> B
    E --> F
    
    G --> B
    G --> F
    G --> C
    
    H --> B
    H --> C
    
    K --> F
    K --> B
    
    L --> B
    L --> F