# Dataset Aggregation {#embodied_il_sim_dagger status=ready}

This section describes the current `ente` DAgger baseline for the lane-following challenge.

The baseline now trains and evaluates against Duckiematrix rather than the older Gym Duckietown simulator, while still following the same high-level DAgger idea of iteratively mixing expert actions and learner actions.


<div class='requirements' markdown='1'>

Requires: You are somewhat familiar with PyTorch and the [Pytorch template](#pytorch-template).

Result: You can train, test, and submit the current DAgger baseline for `aido-LF-sim-validation`.

</div>

## Introduction

We saw a first implementation of imitation learning in the behaviour cloning baseline. 
That baseline models the driving task as an end-to-end supervised learning problem where data can be collected offline from an expert. One of the central issues with this approach is that of **distributional shift**. Since this is a sequential decision making problem, the training data are not "identically and independently distributed". The result is that if your agent deviates from the *optimal* trajectory that was demonstrated by the expert, it will not have any data in its dataset that shows it how to *recover back* to the optimal trajectory. As a result, it is unlikely that the behiaviour cloning approach will be robust.  

For a better result than behaviour cloning, this second version of imitation learning does not train only on a single trajectory given by the expert. We follow the Dataset Aggregation algorithm [(Dagger)](https://www.cs.cmu.edu/~sross1/publications/Ross-AIStats11-NoRegret.pdf), where the learner also interacts with the environment and the expert recovers from the learner's mistakes.

## Quickstart 

Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-dagger-pytorch):

    $ git clone https://github.com/duckietown/challenge-aido_LF-baseline-dagger-pytorch.git
    
Change into the directory:

    $ cd challenge-aido_LF-baseline-dagger-pytorch

The repository already includes `submission.yaml` for `aido-LF-sim-validation` with protocol `aido6_embodied_sys`.

Then test the submission, either locally with:

    $ dts challenges evaluate --challenge aido-LF-sim-validation

or make an official submission when you are ready with 

    $ dts challenges submit

You can find the list of challenges [here][list-challenges]. Make sure that it is marked as "Open". 


[list-challenges]: https://staging-challenges.duckietown.com/humans/challenges


## Local Development Workflow

The repository ships a working model, but the expectation is that you will retrain and improve it.

### Training

Before starting local training, follow the shared Duckiematrix workflow in
[Using Duckiematrix Locally](../60_manual/32_simulator.md). That page documents the engine
and renderer setup, when to use DTPS or SHM, and how to run the same workflow
on a workstation with a display or on a headless GPU host.

From the repository root, run:

    $ python -m training.train
    $ python -m training.test --model-path ![PATH_TO_MODEL]

The training helpers in `training/utils/environment.py` use `gym_duckiematrix.db21j_env.DuckiematrixDB21JEnv`, so the learning path now matches the same Duckiematrix world model used by the runtime submission.


### Parameters that can affect training

There are several optional flags you can use to modify the training run:

* `--episode` or `-i` an integer specifying the number of episodes to train the agent, defaults to 10.
* `--horizon` or `-r` an integer specifying the length of the horizon in each episode, defaults to 64.
* `--learning-rate` or `-l` integer specifying the index from the list [1e-1, 1e-2, 1e-3, 1e-4, 1e-5] to select the learning rate, defaults to 2.
* `--decay` or `-d` integer specifying the index from the list [0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95] to select the initial probability to choose the teacher, the learner.
* `--save-path` or `-s` string specifying the path where to save the trained model, models will be overwritten to keep latest episode, defaults to a file named iil_baseline.pt on the project root.
* `--map-name` or `-m` string  specifying which map to use for training, defaults to loop_empty.
* `--num-outputs` integer specifying the number of outputs the model will have, can be modified to train only angular speed, defaults to 2 for both linear and angular speed.
* `--domain-rand` or `-dr` a flag to enable domain randomization for the transferability to real world from simulation.
* `--randomize-map` or `-rm` a flag to randomize training maps on reset.


The baseline model is based on Dronet. The feature extractor is frozen while the regression head is adapted to the lane-following control task.

### Runtime structure

The `ente` DAgger repository follows the same runtime layout as the template and RL baseline:

- `config.yaml` carries the exercise metadata.
- `solution/` contains the runtime submission payload.
- `models/` contains the shipped inference checkpoint.
- `training/` contains the baseline-owned trainer and test harness.

At runtime, `solution/main.py` talks to the live Duckiematrix evaluator through `gym_duckiematrix.gym_environment.GymEnvironment`. Unlike the ROS repositories, this baseline does not need `launchers/` or `assets/` because it runs a direct Python entrypoint rather than a ROS graph.

### Local evaluation

A simple local test run remains available through `python -m training.test --model-path ![PATH_TO_MODEL]`. Once the runtime checkpoint looks good locally, use the normal `dts challenges evaluate` and `dts challenges submit` commands from the repository root.
   
### Expected Results

The following video shows the results for training the agent during 130 episodes and keeping the rest of the configuration to its default:

<div figure-id="fig:dagger_result">
<a href="https://youtu.be/--Cy_EgdrvU">
<img src="images/dagger_vid.png" style="width: 80%"/>
</a>
</div>


### Tips to Improve your model

Some ideas on how to improve on the provided baseline:

* Map randomization.
* Domain randomization.
* Better selection than random when switching between expert and learner actions.
* Balancing the loss between going straight and turning.
* Change the task from linear and angular speed to left and right wheel velocities.
* Improving the teacher.

## References


``` 

@phdthesis{diaz2018interactive,
  title={Interactive and Uncertainty-aware Imitation Learning: Theory and Applications},
  author={Diaz Cabrera, Manfred Ramon},
  year={2018},
  school={Concordia University}
}

@inproceedings{ross2011reduction,
  title={A reduction of imitation learning and structured prediction to no-regret online learning},
  author={Ross, St{\'e}phane and Gordon, Geoffrey and Bagnell, Drew},
  booktitle={Proceedings of the fourteenth international conference on artificial intelligence and statistics},
  pages={627--635},
  year={2011}
}

@article{loquercio2018dronet,
  title={Dronet: Learning to fly by driving},
  author={Loquercio, Antonio and Maqueda, Ana I and Del-Blanco, Carlos R and Scaramuzza, Davide},
  journal={IEEE Robotics and Automation Letters},
  volume={3},
  number={2},
  pages={1088--1095},
  year={2018},
  publisher={IEEE}
}
```
