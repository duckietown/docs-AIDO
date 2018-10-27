# Tensorflow template for `aido1_LF1` {#tensorflow-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using [TensorFlow](https://www.tensorflow.org/)

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission and see your entry [here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1_r3-v3).

</div>


## Quickstart

1. Clone the [template repo](https://github.com/duckietown/challenge-aido1_LF1-template-tensorflow):

    $ git clone git@github.com:duckietown/challenge-aido1_LF1-template-tensorflow.git


2. Change in the `submission` dir:

    $ cd challenge-aido1_LF1-template-tensorflow/submission
    
3. And run the submission:

    $ dts challenges submit

4. You should be able to see your submission [here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1_r3-v3). 

## Anatomy of the submission

There are two folders:

    learning/
    submission/
    
The `learning` folder describes how to run the learning;
the `submission` part 


## Submission

In the `submission/` directory you will find:

    tf_models/
    model.py
    
The directory `tf_models/` contains the Tensorflow learned models,
produced by the learning step described below.

The `model.py` code is the code that runs the Tensorflow model


## Learning {#tf-learning status=draft}

TODO: for Manfred to write
