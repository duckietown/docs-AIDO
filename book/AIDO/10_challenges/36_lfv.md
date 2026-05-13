# Challenge `LFV` {#challenge-LFV status=ready}

This page documents the historical "lane following with dynamic vehicles" (`LFV`) challenge from earlier AI-DO releases.

<div figure-id="fig:lane-following-vehicles" figure-caption="A Duckiebot doing lane following with other vehicles.">
  <img src="lfv-mixed-dbs.jpg" style='width:100%;height:auto'/>
</div>

Conceptually, `LFV` extends [Challenge `LF`](#challenge-LF) by introducing traffic interactions with other vehicles and static obstacles. In the current `ente` workspace, however, the maintained public workflow focuses on Duckiematrix lane following through `aido-LF-sim-validation`.

## `LFV` in Simulation {#challenge-aido2_lfv status=ready}

There is no maintained public `ente` LFV simulation queue in this workspace.

<div figure-id="fig:submission-output-lfv" figure-caption="Historical visual output for a LFV submission.">
  <img src="lfv-output.png" style='width:100%;height:auto'/>
</div>

If you are working on the current public staging stack:

- start from [Challenge `LF`](#challenge-LF)
- use the [PyTorch template](#pytorch-template) or the [ROS template](#ros-template)
- validate with `dts challenges evaluate --challenge aido-LF-sim-validation`

## Templates and Baselines {status=ready}

The maintained repo set in this workspace provides current LF-only starting points:

- [PyTorch template](#pytorch-template)
- [ROS template](#ros-template)
- [Duckietown ROS baseline](#ros-baseline)
- [reinforcement learning baseline](#embodied_rl)
- [DAgger baseline](#embodied_il_sim_dagger)

If you want to experiment privately with multi-vehicle behavior, adapt one of these current runtimes rather than following the historical LFV submission instructions from older AI-DO releases.

### `aido-LFV-sim-testing` Details {#aido-LFV-sim-testing status=ready}

Historical note: the old LFV testing queue lived on the legacy v4 challenge server and is not part of the maintained `ente` public flow.

### `aido-LFV-sim-validation` Details {#aido-LFV-sim-validation status=ready}

Historical note: the old LFV validation queue is retained here for context only. There is no active staging equivalent in this workspace.

## `LFV` in the Duckietown Autolab {#challenge-aido_lfv-real status=ready}

There is no maintained public `ente` real-robot LFV queue in this workspace.

If you need a robot-side experiment, start from the [ROS template](#ros-template) or the [Duckietown ROS baseline](#ros-baseline) and manage deployment outside the public staging submission path.

### `aido-LFV-real-validation` Details {#aido-LFV-real-validation status=ready}

Historical note: the legacy LFV real-robot queue is not part of the maintained `ente` public workflow.



 












 













