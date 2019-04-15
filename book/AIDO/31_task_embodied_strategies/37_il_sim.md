# Imitation Learning from Simulation {#embodied_il_sim status=draft}

This section describes the procedure for generating logs from the [gym-duckietown](https://github.com/duckietown/gym-duckietown), and then using them to train a model with imitation learning using [tensorflow](https://www.tensorflow.org/). It can be used as a starting point for any of the [`LF`](#lf), [`LFV`](#lf_v), and [`LFVI`](#lf_v_i) challenges.


<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [Tensorflow template](#tensorflow-template) and you understand how it works.

Requires: You already know something about Tensorflow.

Result: You could win the AI-DO!

</div>

## Quickstart 

### Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-IL-sim-tensorflow)

    $ git clone git@github.com:duckietown/challenge-aido_LF-baseline-IL-sim-tensorflow.git
    
### Change into the directory you cloned

    $ cd challenge-aido_LF-baseline-IL-sim-tensorflow
    
### Evaluate your submission

Either locally with 

    $ dts challenges evaluate
    
Or make an official submission

    $ dts challengs submit
    

## How to Improve your Submission


###  Logging

Most of of the logging procedure is implemented on `learning/log.py` and `learning/_loggers.py`.
There are two crucial aspects that can impact your final results:

1. The quality of the expert.
2. The number and variety of samples.

The performance of pure pursuit controller implemented on `teacher.py` is not precisely great.
Even though it uses the internal state of the environment to compute the appropriate action, there are several parameters that need to be fine tuned.
We have prepare some basic debugging capabilities (the `DEBUG=False` flag, line HERE) to help you debug this implementation.
In any case, feel free to provide an expert implementation of your own creation.

Another important aspect you need to take care of is the number of samples.
The number of samples logged are controlled by the `EPISODES` and `STEPS` parameters.
Obviously, the bigger these numbers are, the more samples you get.

As we are using Deep Learning here, the amount of data is crucial, but so it is the variety of the samples we see.
Remember, we are estimating a policy, so the better we capture the underlying distribution of the data, the more robust our policy is.


##  Training

The output of the logging procedure is a file that we called `train.log`, but you can rename it to your convenience.
We have prepared a very simple `Reader` class in `learning/_loggers.py` that is capable of reading the logs we store in the previous step.

The training procedure implemented in `learning/train.py` is quite simple.
The baseline CNN is a one Residual module (Resnet1) network trained to regress the velocity and the steering angle of our simulated duckiebot.
All the Tensorflow boilerplate code is encapsulated in `TensorflowModel` class implemented on `learning/model.py`.
You may find this abstraction quite useful as it is already handling model initialization and persistence ofr you.

Then, the training procedure is quite clear.
We trained the model for a number of `EPOCHS`, using `BATCH_SIZE` samples at each step and the model is persisted every 10 episodes of training.


##  Local Evaluation (Optional)

A simple evaluation script `eval.py` is provided with this implementation.
It loads the latest model from the `trained_models` directory and runs it on the simulator.
Although it is not an official metric for the challenge, you can use the cumulative reward emitted by the `gym` to evaluate the performance of your latest model.

With the current implementation and hyper-parameters selection, we get something like:

```
total reward: -5646.22255589, mean reward: -565.0
```

which is not by any standard a good performance.



