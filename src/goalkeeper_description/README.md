# Goalkeeper Description

Status: an ament_cmake description package with a generated human-shaped URDF. A joint execution controller is not implemented.

The model has slide_joint translating along +Y and tilt_joint rotating about +X. Eleven boxes represent the head, neck, torso, arms, gloves, legs, and feet. Their collision dimensions match the planner's occupied footprint in config/keeper.design.json.

The nominal body is 1.90 m tall with a 1.90 m arm span. It uses a 1.00 m hip-like pivot, 0.05 m upright foot clearance, and approximate 85 kg simulation inertia. These are modelling choices, not FIFA requirements or human biomechanics. Run python3 tools/generate_geometry.py from the repository root after changing the configuration.

See the [architecture](../../docs/architecture.md) and [coordinate conventions](../../docs/interfaces.md).
