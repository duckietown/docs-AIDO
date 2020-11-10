# Dataset Aggregation {#embodied_il_sim_dagger status=ready}

This section describes the procedure for training and testing an agent with the [gym-duckietown](https://github.com/duckietown/gym-duckietown) simulator using the [Dagger](https://www.cs.cmu.edu/~sross1/publications/Ross-AIStats11-NoRegret.pdf) algorithm.

It can be used as a starting point for any of the [`LF`](#challenge-lf), [`LFP`](#challenge-LFP), and [`LFV_multi`](#challenge-LFV_multi) challenges.


<div class='requirements' markdown='1'>

Requires: You are somewhat familiar with PyTorch and the [Pytorch template](#pytorch-template).

Result: You could win the AI-DO!

</div>

## Introduction

We saw a first implementation of immitation learning in the behaviour cloning baseline. That baseline models the driving task as an end-to-end supervised learning problem where data can be collected offline from an expert. One of the central issues with this approach is that of **distributional shift**. Since this is a sequential decision making problem, the training data are not "identically and independently distributed". The result is that if your agent deviates from the *optimal* trajectory that was demonstrated by the expert, it will not have any data in its dataset that shows it how to *recover back* to the optimal trajectory. As a result, it is unlikely that the behiaviour cloning approach will be robust.  

For a better result than behaviour cloning this second version of imitation learning does not train only on a single trajectory given by the expert. We follow the Dataset Aggreagation algorithm [(Dagger)](https://www.cs.cmu.edu/~sross1/publications/Ross-AIStats11-NoRegret.pdf) where we also let the agent interact with the environment and allow the expert to *recover*.  The actions between the expert and the learner are chosen randomly with a varying probability with the hope that the expert _corrects_ the learner if it starts deviating from the optimal trajectory.

## Quickstart 

Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-dagger-pytorch):

    $ git clone https://github.com/duckietown/challenge-aido_LF-baseline-dagger-pytorch.git
    
Change into the directory:

    $ cd challenge-aido_LF-baseline-dagger-pytorch
    
In here you will see two directories `submission` and `learning`. To make a submission, enter the `submission` folder:

    $ cd submission

Then test the submission, either locally with:

    $ dts challenges evaluate --challenge ![CHALLENGE_NAME]

or make an official submission when you are ready with 

    $ dts challenges submit ![CHALLENGE_NAME]

You can find the list of challenges [here][list-challenges]. Make sure that it is marked as "Open". 


[list-challenges]: https://challenges.duckietown.org/v4/humans/challenges


## Local Development Workflow

The previous submission used a model which is included in the repo, but you should try to improve upon it. 

### Option 1: Training with Collab

We provide a [Collab notebook that you can used to get started](https://colab.research.google.com/github/duckietown/challenge-aido_LF-baseline-dagger-pytorch/blob/main/notebook.ipynb) 


During training the loss curve for each episode is available (by default on a folder created on root called `iil_baseline`) and may be checked using `tensorboard` and specifying the `--logidr`. On the same folder you will have `data.dat` and `target.dat` which are the memory maps used by the dataset.


### Option 2: Training Locally


Start by cloning  [the gym-duckietown simulator repo](https://github.com/duckietown/gym-duckietown):

    $ git clone https://github.com/duckietown/gym-duckietown.git
    
Change into the directory:

    $ cd gym-duckietown
    
Install the package:

    $ pip3 install -e .


To run the baseline training procedure, run:

    $ python -m learning.train

in the root directory. 


### Parameters that can affect training


There are several optional flags that may be used to modify hyperparameters of the algorithm:

* `--episode` or `-i` an integer specifying the number of episodes to train the agent, defaults to 10.
* `--horizon` or `-r` an integer specifying the length of the horizon in each episode, defaults to 64.
* `--learning-rate` or `-l` integer specifying the index from the list [1e-1, 1e-2, 1e-3, 1e-4, 1e-5] to select the learning rate, defaults to 2.
* `--decay` or `-d` integer specifying the index from the list [0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95] to select the initial probability to choose the teacher, the learner.
* `--save-path` or `-s` string specifying the path where to save the trained model, models will be overwritten to keep latest episode, defaults to a file named iil_baseline.pt on the project root.
* `--map-name` or `-m` string  specifying which map to use for training, defaults to loop_empty.
* `--num-outputs` integer specifying the number of outputs the model will have, can be modified to train only angular speed, defaults to 2 for both linear and angular speed.
* `--domain-rand` or `-dr` a flag to enable domain randomization for the transferability to real world from simulation.
* `--randomize-map` or `-rm` a flag to randomize training maps on reset.


The baseline model is based on the Dronet model. The feature extractor of the model is frozen while the classifier is modified for the regression task.

All the PyTorch boilerplate code is encapsulated in the `NeuralNetworkPolicy` class implemented on `learning/imitation/iil-dagger/learner/neural_network_policy.py`and is based on previous work done by Manfred Díaz on Tensorflow.


###  Local Evaluation

A simple testing script `test.py` is provided with this implementation.
It loads the latest model from the the provided directory and runs it on the simulator. To test the model:
    
    $ python -m learning.test --model-path ![path]
    
The model path flag has to be provided for the script to load the model:

* `--model-path` or `-mp` string specifying the path to the saved model to be used in testing.

Other optional flags that may be used are:

* `--episode` or `-i` an integer specifying the number of episodes to test the agent, defaults to 10.
* `--horizon` or `-r` an integer specifying the length of the horizon in each episode, defaults to 64.
* `--save-path` or `-s` string specifying the path where to save the trained model, models will be overwritten to keep latest episode, defaults to a file named iil_baseline.pt on the project root.
* `--num-outputs` integer specifying the number of outputs the model has, defaults to 2.
* `--map-name` or `-m` string  specifying which map to use for training, defaults to loop_empty.
   
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
* Better selection than random when switching between expert/learner actions.
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
