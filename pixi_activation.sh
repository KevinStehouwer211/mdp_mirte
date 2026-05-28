#!/usr/bin/env bash

# Keep this file for Pixi-specific environment tweaks.
# Do not prepend ${CONDA_PREFIX}/lib globally here: it makes system tools such
# as /usr/bin/cmake pick up Pixi libraries like libcurl. TypeDB-based ROS nodes
# add the path locally in their wrapper scripts instead.
