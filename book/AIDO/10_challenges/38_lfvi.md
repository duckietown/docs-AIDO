# Challenge `LFVI-multi-full`  {#challenge-LFVI-multi-stateful status=ready}

This page documents the historical multi-agent, full-state `LFVI` challenge from earlier AI-DO releases.

<div figure-id="fig:lane-following-vehicles-intersections-LFVI" figure-caption="A Duckiebot following a lane following in the presence of other vehicles, in a Duckietown with intersections.">
  <img src="lfvi-no-tls.jpg" style='width:100%;height:auto'/>
</div>

This variant combined intersections, dynamic vehicles, and a multi-agent setup in which the submitted policy controlled all Duckiebots with additional state information. The current `ente` workspace does not publish a maintained public queue or maintained template for this protocol.

## `LFVI_multi_full` in Simulation {#challenge-aido_lfvi status=ready}

There is no maintained public `ente` simulation queue for this `LFVI` variant in this workspace.

<div figure-id="fig:submission-output-lfvi" figure-caption="Historical visual output for a LFVI submission.">
  <img src="lfvi-output.png" style='width:100%;height:auto'/>
</div>

If you are working today on the maintained stack, start from [Challenge `LF`](#challenge-LF) and a current runtime such as the [PyTorch template](#pytorch-template) or [ROS template](#ros-template). Private research extensions can build on those repos, but the historical full-state multi-agent public flow is not active here.

## Templates {status=ready}

## Templates and Baselines {status=ready}

The maintained repo set in this workspace does not include a dedicated `LFVI` full-state template. Use the current LF templates or baselines as the runtime surface if you want to prototype related ideas privately.

### `aido-LFVI_multi-sim-testing` Details {#aido-LFVI_multi-sim-testing status=ready}

Historical note: the old `LFVI` testing queue belonged to the legacy v4 challenge server and is not part of the maintained `ente` public flow.

### `aido-LFVI_multi-sim-validation` Details {#aido-LFVI_multi-sim-validation status=ready}

Historical note: the old `LFVI` validation queue is retained here for context only. There is no active staging equivalent in this workspace.


