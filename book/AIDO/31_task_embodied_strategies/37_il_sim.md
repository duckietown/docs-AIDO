# Imitation Learning from Simulation {#embodied_il_sim status=draft}

This section describes the procedure for generating logs from the [gym-duckietown](https://github.com/duckietown/gym-duckietown), and then using them to train a model with imitation learning using [tensorflow](https://www.tensorflow.org/). It can be used as a starting point for any of the [`LF`](#lf), [`LFV`](#lf_v), and [`LFVI`](#lf_v_i) challenges.


<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [Tensorflow template](#tensorflow-template) and you understand how it works.

Requires: You already know something about Tensorflow.

Result: You could win the AI-DO!

</div>

## Quickstart 

1) Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-IL-sim-tensorflow)

    $ git clone git@github.com:duckietown/challenge-aido_LF-baseline-IL-sim-tensorflow.git
    
2) Change into the directory:

    $ cd challenge-aido_LF-baseline-IL-sim-tensorflow
    
3) Install this package

    $ pip3 install -e . # if you are in a conda env
    $ sudo pip3 install -e .  # if you want to install this system-wide

and install gym-duckietown (Use `sudo` if system-wide)

    $ pip3 install -e git://github.com/duckietown/gym-duckietown.git#egg=gym-duckietown
    
(4) Start training (see below)
    

## How to Improve your Submission

You will find that most of the new code sits inside of the `learning/` subdirectory. If you've been following along, or have looked at other template or baseline repositories for AIDO, you will find that many of the files in `submission/` are the same as other repositories in the Duckietown Universe.

Here, we'll be focusing on _imitation learning_, or learning a policy from a set of expert trajectories. In the following sections, we'll cover both how to retrieve those training trajectories, as well as learn a policy from them. In each section, we will give hints and debugging tips on how to improve your submission to the baseline we've provided here.

###  Logging

To run and log the _baseline_ expert, you can run:

    $ python log.py
    
within the `learning/` directory. Of course, if you are just interested in the baseline and seeing how this all works together, you can skip to the next section.

Most of of the logging procedure is implemented on `learning/log.py` and `learning/_loggers.py`. This logging will run a hard coded expert on a variety of `gym-duckietown` maps, and record the actions it takes. 

To improve from our baseline, you will need to focus on two crucial aspects:

1. The quality of the expert.
2. The number and variety of samples.

The performance of pure pursuit controller implemented on `teacher.py` can definitely be improved upon.
Even though it uses the internal state of the environment to compute the appropriate action, there are several parameters that need to be fine tuned.

We have prepare some basic debugging capabilities (the `DEBUG=False` flag, line HERE) to help you debug this implementation.
In any case, feel free to provide an expert implementation of your own creation.

Another important aspect you need to take care of is the number of samples.
The number of samples logged are controlled by the `EPISODES` and `STEPS` parameters.
Obviously, the bigger these numbers are, the more samples you get.

As with all Deep Learning methods, the amount of data is crucial, but so is the variety of the samples we see.
Remember, we are estimating a policy, so the better we capture the underlying distribution of the data, the more robust our policy will be.


##  Training

In imitation learning, given the expert trajectories (which we recorded in the previous section), we want to learn a policy that _imitates_ those trajectories. To do this, we will use a naive method called behavior cloning (BC) - BC has many issues, but we will leave it up to you to figure out what those are!

The output of the logging procedure is a file that we called `train.log`, but you can rename it to your convenience.
We have prepared a very simple `Reader` class in `learning/_loggers.py` that is capable of reading the logs we store in the previous step.

To run the baseline training procedure, run:

    $ python train.py
    
in the `learning/` directory. 

The training procedure implemented in `learning/train.py` is relatively simple, compared to many of today's state-of-the-art imitation learning systems.

The baseline CNN is a one Residual module (Resnet1) network trained to regress the velocity and the steering angle of our simulated Duckiebot.

All the Tensorflow boilerplate code is encapsulated in `TensorflowModel` class implemented on `learning/model.py`.
You may find this abstraction quite useful as it is already handling model initialization and persistence for you.

To summarize the code in `train.py`, we train the model for a number of `EPOCHS`, using `BATCH_SIZE` samples at each step  to regress the steering and velocity from each input image. The model is saved every 10 episodes of training.

Of course, this can be improved upon in many ways, on both the teacher / expert side, as well as the student policy side. 

## Evaluate your submission

You will need to copy the relevant files from the `learning/` directory to the `submission/` one. In particular, you will need to overwrite `submission/model.py` to match any update you've made to the model, and place your final model inside of `submission/tf_models/` so you can load it correctly. Then, you are ready to evaluate!

Either locally with 

    $ dts challenges evaluate
    
Or make an official submission

    $ dts challengs submit



##  Local Evaluation (Optional)

A simple evaluation script `eval.py` is provided with this implementation.
It loads the latest model from the `trained_models` directory and runs it on the simulator.
Although it is not an official metric for the challenge, you can use the cumulative reward emitted by the `gym` to evaluate the performance of your latest model.

With the current implementation and hyper-parameters selection, we get something like:

```
total reward: -5646.22255589, mean reward: -565.0
```

which is not by any standard a good performance.

## Sim2Real Transfer (Optional)

Doing great on the simulated challenges, but not on the real evaluation? Or doing great in your training, but not on our simulated, held-out environments? Take a look at `env.py`. You'll notice that we launch the `Simulator` class from `gym-duckietown`. When we [take a look at the constructor](https://github.com/duckietown/gym-duckietown/blob/aido2_lf_r1/gym_duckietown/simulator.py#L145-L180), you'll notice that we aren't using all of the parameters listed. In particular, the three you should focus on are:
    
- `map_name`: What map to use; hint, take a look at gym_duckietown/maps for more choices
- `domain_rand`: Applies domain randomization, a popular, black-box, sim2real technique
- `randomized_maps_on_reset`: Slows training time, but increases training variety.

Mixing and matching different values for these will help you improve your training diversity, and thereby improving your evaluation robustness!

If you're interested in more advanced techniques, like learning a representation that is a bit easier for your network to work with, or one that transfers better across the simulation-to-reality gap, there are some [alternative, more advanced methods](https://github.com/duckietown/segmentation-transfer) you may be interested in trying out. In addition, don't forget to try using the [logs infrastructure](http://logs.duckietown.org/), which you can also use to do things like [imitation learning](https://github.com/duckietown/challenge-aido_LF-baseline-IL-logs-tensorflow/)!

