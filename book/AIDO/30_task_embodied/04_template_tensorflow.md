# TensorFlow Template  {#tensorflow-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using [TensorFlow](https://www.tensorflow.org/). 
It can be used as a starting point for any of the [`LF`](#challenge-LF), [`LFV`](#challenge-LFV), and [`LFI`](#challenge-LFI) 
challenges.

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission to all of the `LF*` challenges and can 
view their status and output.

</div>

<figure id="aido-webinar-tensorflow">
    <figcaption>TensorFlow Template</figcaption>
    <dtvideo src="vimeo:481632757"/>
</figure>

## Quickstart

Clone the [template repo](https://github.com/duckietown/challenge-aido_LF-template-tensorflow):

    $ git clone git@github.com:duckietown/challenge-aido_LF-template-tensorflow.git


Change into the directory:

    $ cd challenge-aido_LF-template-tensorflow
    

Either make a submission with:

    $ dts challenges submit --challenge ![CHALLENGE_NAME]
    
where you can find a list of the open challenges [here](https://challenges.duckietown.org/v4/humans/challenges).


Or, run local evaluation with:

    $ dts challenges evaluate --challenge ![CHALLENGE_NAME]

### Verify your submission(s)

This will make a number of submissions (as described below). You can track the status of these submissions in the command line with:

    $ dts challenges follow --submission ![SUBMISSION_NUMBER]

or through your browser by navigating the webpage: `https://challenges.duckietown.org/v4/humans/submissions/![SUBMISSION_NUMBER]`

where `![SUBMISSION_NUMBER]` should be replaced with the number of the submission which is reported in the terminal output. 

## Anatomy of the submission


The submission consists of all of the basic files that required for a [basic submission](#minimal-template). Below we will highlight the specifics with respect to this template. 


### `solution.py`

The only difference in `solution.py` is that we are initializing our model:

```python
    from model import TfInference
    # define observation and output shapes
    self.model = TfInference(observation_shape=(1,) + expect_shape,
                                 # this is the shape of the image we get.
                                 action_shape=(1, 2),  # we need to output v, omega.
                                 graph_location='tf_models/')  # this is the folder where our models are stored.
    self.current_image = np.zeros(expect_shape)
```

and then we call our model to compute an action with the following code:

```python
    def compute_action(self, observation):
        action = self.model.predict(observation)
        return action.astype(float)
```

Note that we also can require the presence of a GPU with the environment variable `AIDO_REQUIRE_GPU` and then the solution will fail if a GPU is not found. 

### Model files

The other additional files are the following:

    tf_models/
    model.py
    
The directory `tf_models/` contains the Tensorflow learned models (the ones that you have trained).

The `model.py` code is the code that runs the Tensorflow model.

