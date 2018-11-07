# Tensorflow template for `aido1_LF1` {#tensorflow-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using [TensorFlow](https://www.tensorflow.org/)

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission and see your entry [here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1_r3-v3).

</div>


## Quickstart

### Clone the [template repo](https://github.com/duckietown/challenge-aido1_LF1-template-tensorflow):

    $ git clone git@github.com:duckietown/challenge-aido1_LF1-template-tensorflow.git


### Change in the `submission` dir:

    $ cd challenge-aido1_LF1-template-tensorflow
    
### And run the submission:

    $ dts challenges submit

### You should be able to see your submission [here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1_r3-v3). 

## Anatomy of the submission

The submission consists of all of the basic files that required for a [basic submission](#challenge-aido1_lf1-template-random).

### solution.py

The `solution.py` differs from the basic version in the following important was. First, the model is instantiated with:

```python
    from model import TfInference
    # define observation and output shapes
    model = TfInference(observation_shape=(1,) + expect_shape,  # this is the shape of the image we get.
                        action_shape=(1, 2),  # we need to output v, omega.
                        graph_location='tf_models/')  # this is the folder where our models are st
```

Then in the infinite loop we have to run inference on the model:

```python
    # we passe the observation to our model, and we get an action in return
    action = model.predict(observation)
```


### Model files

The other additional files are the following:

    tf_models/
    model.py
    
The directory `tf_models/` contains the Tensorflow learned models.

The `model.py` code is the code that runs the Tensorflow model

For some options for how to train these models you can check out [](#embodied_il_sim) or [](#embodied_il_logs).
