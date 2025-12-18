# Mechanical Motion & Fit Plugin (`mechfit`)

This plugin adds mechanical reasoning capabilities to the Fabrikator CAD agent. It allows the agent to validate movement, check for collisions, and ensure parts fit together correctly before manufacturing.

## Core Concept: Anchor-Based Assembly
Unlike traditional CAD where parts must be exported in perfect global coordinates, this plugin uses a **Feature-Based Assembly** workflow:
1.  **Local Space**: Parts are exported in their own local coordinate space.
2.  **Anchors**: The agent defines "Anchors" (coordinate frames) on key features like holes, hinge axes, or mounting points.
3.  **Joints**: The agent connects parts by linking two anchors. The plugin solves the kinematics to place the parts correctly.

## Tools

### `define_anchor`
Defines a coordinate frame on a part.
- **Inputs**: `part` (name), `origin` (local [x,y,z]), `z_axis` (primary axis), `x_axis` (orientation).
- **Output**: Returns a **Visual Debug Image** showing the coordinate frame overlaid on the part.
- **Usage**: Use the debug image to verify the axis points in the intended direction (e.g., Z-axis along the hinge pin).

### `define_joint`
Connects two parts via their anchors.
- **Inputs**: `parent_part`, `parent_anchor`, `child_part`, `child_anchor`, `type` (revolute/prismatic/fixed), `limits`.
- **Logic**: Moves the `child_part` so its anchor aligns with the `parent_part`'s anchor.

### `motion_sweep_check`
Simulates movement to detect collisions.
- **Inputs**: `joints` (list of joint IDs to actuate).
- **Process**: Sweeps the joint through its `limits` (using `step_deg`) and checks for mesh intersections at each step.

### `insertion_path_check`
(Coming Soon) Checks if a part can be inserted linearly without interference.

## Installation & Development

### Prerequisites
- Docker
- Python 3.11+

### Running Locally
```bash
# Build the container
docker build -t mechfit .

# Run tests
docker run --rm mechfit pytest tests/test_integration.py
```

### Deployment
The repository includes a GitHub Action (`docker-publish.yml`) that automatically builds and pushes the image to GitHub Container Registry (GHCR) on release.

## License
Proprietary / Fabrikator Team.
