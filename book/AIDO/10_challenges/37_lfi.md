# Challenge `LFI` {#challenge-LFI status=ready}

This page documents the historical "lane following with intersections" (`LFI`) challenge from earlier AI-DO releases.

<div figure-id="fig:lane-following-vehicles-intersections-LFI" figure-caption="A Duckiebot following a lane in a Duckietown with intersections.">
  <img src="lfi-db21.jpg" style='width:100%;height:auto'/>
</div>

Conceptually, `LFI` extends [Challenge `LF`](#challenge-LF) by emphasizing intersection traversal. In the current `ente` workspace there is no separate maintained public `LFI` queue, although the maintained `LF` evaluation still reports `intersection_count` as one of its scores.

## `LFI` in Simulation {#challenge-aido_lfi status=ready}

There is no maintained public `ente` simulation queue for `LFI` in this workspace.

<div figure-id="fig:submission-output-lfi" figure-caption="Historical visual output for a LFI submission.">
  <img src="lfi-output.png" style='width:100%;height:auto'/>
</div>

If you are working on the current public staging stack, use [Challenge `LF`](#challenge-LF) and adapt a maintained runtime if you want stronger intersection behavior.

## Templates and Baselines {status=ready}

The maintained repo set in this workspace provides current LF-facing starting points:

- [PyTorch template](#pytorch-template)
- [ROS template](#ros-template)
- [Duckietown ROS baseline](#ros-baseline)
- [reinforcement learning baseline](#embodied_rl)
- [DAgger baseline](#embodied_il_sim_dagger)

### `aido-LFI-sim-testing` Details {#aido-LFI-sim-testing status=ready}

Historical note: the old `LFI` testing queue belonged to the legacy v4 challenge server and is not part of the maintained `ente` public flow.

### `aido-LFI-sim-validation` Details {#aido-LFI-sim-validation status=ready}

Historical note: the old `LFI` validation queue is retained here for context only. There is no active staging equivalent in this workspace.

## `LFI` in the Duckietown Autolab {#challenge-aido_lfi-real status=ready}

There is no maintained public `ente` real-robot `LFI` queue in this workspace.

If you need robot-side experiments with intersections, start from the [ROS template](#ros-template) or the [Duckietown ROS baseline](#ros-baseline) and manage deployment outside the public staging submission path.

### `aido-LFI-real-validation` Details {#aido-LFI-real-validation status=ready}

Historical note: the legacy `LFI` real-robot queue is not part of the maintained `ente` public workflow.


