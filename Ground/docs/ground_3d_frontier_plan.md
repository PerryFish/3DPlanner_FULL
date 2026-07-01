# Ground 3D Frontier Plan

The current ground baseline is not 2D frontier exploration. It reads `/bimodal/map_3d`, samples candidates inside a 3D boundary, applies a ground constraint, checks obstacle clearance in 3D, and scores candidates with unknown gain, distance gain, obstacle penalty, and revisit penalty.

Ground mode is a constraint layer on a shared 3D map, not a separate 2D SLAM Toolbox or 2D occupancy grid pipeline.

Future upgrades can follow TARE/GBPlanner-style ideas:

- local dense frontier extraction
- global sparse graph
- traversability map
- viewpoint utility
- revisit penalty
- stuck recovery
- long-horizon exploration graph
