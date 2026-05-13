# Minimal pure-Python Template {#minimal-template status=ready}

This page describes a historical pure-Python template from earlier AI-DO releases.

The current `ente` repo set in this workspace does not ship a maintained `challenge-aido_LF-template-random` repository. For current submissions use either the [PyTorch template](#pytorch-template) or the [ROS template](#ros-template).

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You know which maintained `ente` template to start from.

</div>

## Current `ente` starting points

For the maintained public lane-following workflow:

- Clone `challenge-aido_LF-template-pytorch` if you want a Python-only runtime that talks directly to Duckiematrix.
- Clone `challenge-aido_LF-template-ros` if you want a ROS graph inside the solution container.
- Use `dts challenges evaluate --challenge aido-LF-sim-validation` for local validation and `dts challenges submit` for public submission.

## Historical note

The older random template targeted the legacy `aido2_db18_agent-z2` protocol and the v4 challenges server. Those instructions are not the active `ente` submission path.
