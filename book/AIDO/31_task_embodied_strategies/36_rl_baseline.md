# Reinforcement Learning {#embodied_rl status=ready}

This section describes the current `ente` reinforcement-learning baseline built on top of the [PyTorch template](#pytorch-template).

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [PyTorch template](#pytorch-template).

Requires: Patience, training RL agents is not easy.

Result: You have a functional agent trained with RL. Your expectations in regards to end-to-end RL's capabilities should be realistic. 

</div>

Before getting started, note that end-to-end RL for lane following is still an iterative research workflow. The shipped baseline is meant to be a working reference point rather than a final policy.


## Quickstart {#rl-baseline-quickstart}


Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-RL-sim-pytorch)

    $ git clone https://github.com/duckietown/challenge-aido_LF-baseline-RL-sim-pytorch.git

Change into the  directory:

    $ cd challenge-aido_LF-baseline-RL-sim-pytorch

The repository already includes `submission.yaml` for `aido-LF-sim-validation`.

Test the submission, either locally with:

    $ dts challenges evaluate --challenge aido-LF-sim-validation

or make an official submission when you are ready with 

    $ dts challenges submit

You can find the list of challenges [here][list-challenges]. Make sure that it is marked as "Open". 


[list-challenges]: https://staging-challenges.duckietown.com/humans/challenges


## How to Train your Policy

Before starting a local training run, follow the shared Duckiematrix setup in
[Using Duckiematrix Locally](../60_manual/32_simulator.md). That page covers the engine and
renderer bring-up, the difference between DTPS and SHM mode, and the extra
steps required on a headless machine.

The `ente` RL baseline keeps its trainer in the `training/` package. From the repository root, use:

    $ python -m training.train
    $ python -m training.test

The training helpers build a `gym_duckiematrix.db21j_env.DuckiematrixDB21JEnv` for `map_0/vehicle_0`, so the training and inference paths match the current Duckiematrix-based evaluation stack instead of the older Gym Duckietown simulator.


## How to submit the trained policy

The runtime image extends `duckietown/challenge-aido_lf-template-pytorch:ente` and keeps the inference path under `solution/`. The shipped checkpoint lives under `models/`, and the entrypoint remains:

    $ python -m solution.main

After you train a better policy, replace the checkpoint in `models/`, update the inference code in `solution/` as needed, and then use the same `evaluate` or `submit` workflow described in the [Quickstart](#rl-baseline-quickstart).




## How to improve your policy

Here are some ideas for improving the baseline:

- Modify the reward shaping in the wrappers under `training/utils/`.
- Resize or normalize the camera observation differently before inference.
- Tune the training hyperparameters exposed by `training/train.py`.
- Swap in a different network architecture inside the baseline policy.
- Randomize the training environment more aggressively if you need more robustness.



## Runtime structure

The RL baseline follows the same high-level structure as the template:

- `config.yaml` carries the exercise metadata.
- `solution/` contains the inference runtime that talks to Duckiematrix through `GymEnvironment`.
- `models/` contains the shipped checkpoint.
- `training/` contains the baseline-owned training and evaluation code.

That one-runtime, one-training-surface split is the main `ente` change from the older RL baseline layouts. Unlike the ROS repositories, this baseline does not need `launchers/` or `assets/` because it runs a direct Python entrypoint rather than a ROS graph.
