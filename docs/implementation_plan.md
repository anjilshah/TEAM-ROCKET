# Implementation phases

1. Inspect Ubuntu, Python tooling, the existing workspace, and applicable local instructions.
2. Locate a licensed `wx250s` MJCF and verify it against official Interbotix parameters. Use the existing Menagerie conversion rather than introducing ROS.
3. Create a reproducible editable Python package, configuration, physical scene, real socket openings, and calibrated RGB-D interfaces.
4. Implement rendered-image segmentation, contour classification, top-face localization, and observed socket detection.
5. Implement model-Jacobian pose IK, local gated gripper attachment, conservative checked trajectories, and complete autonomous sorting.
6. Add keyboard control and stop behavior, physical and two-camera placement verification, bounded recovery, seeded evaluation, and tests.
7. Run complete episodes, fix demonstrated failures, record an actual demo, document assumptions and measurements, and prepare a local Git repository.
8. Request owner approval only after files/results are reviewable, before public repository or video publication.
