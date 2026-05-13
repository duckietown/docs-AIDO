# Imitation Learning from Logs {#embodied_il_logs status=beta}

This page documents a historical TensorFlow imitation-learning-from-logs baseline from earlier AI-DO releases.

The current `ente` repo set in this workspace does not ship a maintained `challenge-aido_LF-baseline-IL-logs-tensorflow` repository. For maintained learning workflows use the [DAgger baseline](#embodied_il_sim_dagger), the [reinforcement learning baseline](#embodied_rl), or the [PyTorch template](#pytorch-template) as the runtime surface you adapt.

<div class='requirements' markdown='1'>

Requires: That you understand the current maintained `ente` learning paths.

Result: You know which maintained `ente` repos replace this historical log-based TensorFlow workflow.

</div>

## Current `ente` workflow

- Keep runtime code inside the maintained `solution/` layout used by the current templates and baselines.
- Treat logged-data learning as an offline training step that produces weights for a current Duckiematrix-facing runtime.
- Validate locally with `dts challenges evaluate --challenge aido-LF-sim-validation` before submitting.

## Historical note

The old log-based baseline depended on TensorFlow-specific tooling, legacy bag-processing scripts, and challenge-server instructions that are not part of the maintained `ente` workspace.
