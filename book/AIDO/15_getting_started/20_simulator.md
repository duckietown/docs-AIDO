# The Duckietown Simulator {#dt-simulator status=beta}

<div figure-id="fig:simplesim_free">
<img src="images/simplesim_free.png" style="width: 80%"/>
</div>

In the current `ente` submission flow, the maintained simulator is Duckiematrix rather than the older standalone `gym-duckietown` quickstart used in earlier AI-DO releases.

## Current workflow

For the maintained lane-following workflow in this workspace:

- Public evaluation runs on Duckiematrix through the challenge `aido-LF-sim-validation`.
- Local evaluation uses the same challenge definition and the same solution image.
- The recommended entrypoint is to run evaluation from one of the maintained submission repos:

    $ dts challenges evaluate --challenge aido-LF-sim-validation

You do not need to install a separate host-side simulator package to validate the current templates or baselines.

## Runtime model

Duckiematrix runs as part of the evaluator stack and injects the connection details into the solution environment:

- `VEHICLE_NAME`
- `DUCKIEMATRIX_ENGINE_HOSTNAME`
- `DUCKIEMATRIX_ENGINE_PORT`
- `DTSHELL_SHM_PATH` when shared-memory transport is enabled

The [PyTorch template](#pytorch-template) and the maintained learning baselines talk to Duckiematrix through `gym-duckiematrix`. The [ROS template](#ros-template) bridges the same evaluator session into ROS topics.

## Maps, observations, and actions

The active public LF evaluation on `ente` uses the loop map in Duckiematrix.

- Observations are front-camera images and related world input keyed by vehicle name.
- Actions are left and right wheel PWM commands in the interval `[-1, 1]`.
- The evaluator advances the simulation when the solution publishes its next action.

## When to use what

- Use `dts challenges evaluate` when you want to validate a submission against the current public challenge contract.
- Use the maintained template and baseline repos in this workspace when you need code examples for the current stack.
- Treat older `gym-duckietown` host-installation instructions elsewhere in the book as historical reference rather than the default `ente` path.

## Historical note

Older AIDO material in this book refers to `gym-duckietown` as a standalone simulator with direct host installation instructions. That material is retained for historical context, but it is not the maintained default path for the current `ente` submission workflow.

If you specifically need to cite the historical `gym-duckietown` simulator project, use the following BibTeX entry:

```
@misc{gym_duckietown,
  author = {Chevalier-Boisvert, Maxime and Golemo, Florian and Cao, Yanjun and Mehta, Bhairav and Censi, Andrea and Paull, Liam},
  title = {Duckietown Environments for OpenAI Gym},
  year = {2018},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/duckietown/gym-duckietown}},
}
```
