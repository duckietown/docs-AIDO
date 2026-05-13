# Behavior Cloning {#embodied_bc status=ready}

This page documents a historical TensorFlow behavior-cloning baseline from earlier AI-DO releases.

The current `ente` repo set in this workspace does not ship a maintained `challenge-aido_LF-baseline-behavior-cloning` repository. For current learning workflows, use the [DAgger baseline](#embodied_il_sim_dagger), the [reinforcement learning baseline](#embodied_rl), or the [PyTorch template](#pytorch-template) as the runtime surface you adapt.

<div class='requirements' markdown='1'>

Requires: That you understand the maintained `ente` learning paths.

Result: You know which maintained `ente` repos replace this historical behavior-cloning baseline.

</div>

## Current alternatives

- Use the [PyTorch template](#pytorch-template) when you want to build your own offline imitation-learning runtime.
- Use the [DAgger baseline](#embodied_il_sim_dagger) when you want a maintained imitation-learning reference implementation.
- Use `dts challenges evaluate --challenge aido-LF-sim-validation` to validate a trained runtime before public submission.

## Historical note

The older behavior-cloning baseline depended on TensorFlow tooling, historical Duckietown logging workflows, and v4 challenge-server examples. It is retained here for historical context only.
