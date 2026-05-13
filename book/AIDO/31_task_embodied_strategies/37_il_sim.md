# Imitation Learning from Simulation {#embodied_il_sim status=beta}

This page documents a historical TensorFlow imitation-learning baseline from earlier AI-DO releases.

The current `ente` repo set in this workspace does not ship a maintained `challenge-aido_LF-baseline-IL-sim-tensorflow` repository. For maintained Duckiematrix-based learning workflows use the [DAgger baseline](#embodied_il_sim_dagger), the [reinforcement learning baseline](#embodied_rl), or the [PyTorch template](#pytorch-template) as your starting point.

<div class='requirements' markdown='1'>

Requires: That you understand the current maintained `ente` learning paths.

Result: You know which maintained baselines replace this historical TensorFlow workflow.

</div>

## Current `ente` workflow

- Train and test from the `training/` packages in the maintained baselines that ship in this workspace.
- Validate locally with `dts challenges evaluate --challenge aido-LF-sim-validation`.
- Submit with `dts challenges submit` once the runtime image behaves correctly.

## Historical note

The old simulation imitation-learning baseline depended on TensorFlow, `gym-duckietown`, and a repo branch that is not part of the maintained `ente` workspace.

