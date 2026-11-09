# Goalkeeper Description

The generated [URDF](urdf/goalkeeper.urdf) and [closed diamond mesh](meshes/diamond_panel.stl) share [keeper.design.json](../../config/keeper.design.json). The whole assembly has one lateral `slide_joint`. The solid diamond is 1.45 m × 0.90 m across its diagonals, with its upper tip 2.15 m above the pitch.

Regenerate via `python3 tools/generate_geometry.py` and rebuild. The mesh is installed in the description package; the launch supplies its parent share path for [Gazebo package URI resolution](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/README.md). See [dimensions and validation](../../docs/robot-design.md).

The goalkeeper portrait is mapped across both plate fronts with a visual-only [OBJ decal](meshes/keeper-front-decal.obj) and [print texture](meshes/keeper-front-print.png). The collision surfaces remain the original shapes. The mesh uses [OBJ and MTL texture mapping supported by Gazebo](https://gazebosim.org/api/sim/8/migrationsdf.html); its appearance in the live simulator remains to be checked.
