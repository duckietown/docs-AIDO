# Residual Policy Learning {#embodied_rpl status=ready}

This page documents a historical residual-policy-learning baseline from earlier AI-DO releases.

The current `ente` repo set in this workspace does not ship a maintained `challenge-aido_LF-baseline-RPL-ros` repository. If you want a similar workflow on the current stack, combine the [Duckietown ROS baseline](#ros-baseline) with ideas from the [reinforcement learning baseline](#embodied_rl) in your own maintained repository.

<div class='requirements' markdown='1'>

Requires: That you understand the maintained ROS and RL baselines.

Result: You know how this historical RPL page maps onto the current `ente` repo set.

</div>

## Current alternatives

- Start from the [Duckietown ROS baseline](#ros-baseline) if you want the maintained classical lane-following stack.
- Use the [reinforcement learning baseline](#embodied_rl) when you want a maintained learning baseline that already targets Duckiematrix.
- Port residual-policy ideas into a current repo only after the base runtime works with `aido-LF-sim-validation`.

## Historical note

The older RPL baseline depended on a dedicated ROS+RL repository, `gym-duckietown` training glue, and challenge-server instructions that are not part of the maintained `ente` workspace.
