# Challenge `LFV_multi` {#challenge-LFV_multi status=draft}

This page documents the historical multi-agent lane-following-with-vehicles (`LFV_multi`) challenge from earlier AI-DO releases.

<div figure-id="fig:lane-following-vehicles-LFV_multi" figure-caption="A Duckiebot doing lane following with other vehicles. In this _multi_ variant, the submitted agent runs on all Duckiebots.">
  <img src="lfv-db19.jpg" style='width:100%;height:auto'/>
</div>

In this variant the submitted policy embodied all vehicles in the scene. The current `ente` workspace does not publish a maintained public `LFV_multi` queue or a maintained multi-agent template for it.

## `LFV_multi` in Simulation {#challenge-aido_lfv_multi status=ready}

There is no maintained public `ente` simulation queue for `LFV_multi` in this workspace.

<div figure-id="fig:submission-output-lfv_multi" figure-caption="Historical visual output for a LFV-multi submission.">
  <img src="lfv-multi-output.png" style='width:100%;height:auto'/>
</div>

If you want to experiment with multi-agent behavior today, start from the maintained LF runtime and extend it privately rather than relying on the historical public challenge setup.

## Templates {status=draft}

The maintained repo set in this workspace does not include a dedicated `LFV_multi` template. Use a current LF template or baseline as the runtime surface and add your own multi-agent coordination logic outside the public staging flow.

### `aido5-LFV_multi-sim-testing` Details {#aido5-LFV_multi-sim-testing status=ready}

Historical note: the old `LFV_multi` testing queue belonged to the legacy v4 challenge server and is not part of the maintained `ente` public flow.

### `aido5-LFV_multi-sim-validation` Details {#aido5-LFV_multi-sim-validation status=ready}

Historical note: the old `LFV_multi` validation queue is retained here for context only. There is no active staging equivalent in this workspace.

## `LFV_multi` in the the Duckietown Autolab {#challenge-aido5_lfv_multi_robotarium status=draft}

There is no maintained public `ente` robot-side `LFV_multi` queue in this workspace.






 















