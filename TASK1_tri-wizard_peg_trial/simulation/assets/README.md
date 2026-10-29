# Scene assets

The four peg meshes and convex socket-wall prisms are generated directly as MJCF vertices by `src/peg_sorting/simulation/scene_manager.py`; they require no external mesh files. Square and circular pegs use primitive collision geometry for stable flat contacts. Triangle and hexagon pegs use convex prism meshes. The circular aperture uses 48 sides.

Robot STL meshes and its texture are stored in `simulation/robots/wx250s/assets/` with upstream licensing and hashes. The nominal scene uses portable relative paths to those files.
