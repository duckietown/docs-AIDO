# PyTorch Template {#pytorch-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using [PyTorch](https://pytorch.org/). 

It can be used as a starting point for any of the [`LF`](#challenge-LF), 
[`LFV_multi`](#challenge-LFV), and [`LFI`](#challenge-LFI) challenges.

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission to all of the `LF*` challenges and can view their status and output.

</div>

<figure id="aido-webinar-pytorch">
    <figcaption>PyTorch Template</figcaption>
    <dtvideo src="vimeo:480202594"/>
</figure>



## Quickstart

Clone the [template repo](https://github.com/duckietown/challenge-aido_LF-template-pytorch):

    $ git clone git://github.com/duckietown/challenge-aido_LF-template-pytorch.git

Change into the directory:
    
    $ cd challenge-aido_LF-template-pytorch 
        
Run the submission:

Either make a submission with:

    $ dts challenges submit --challenge ![CHALLENGE_NAME]
    
where you can find a list of the open challenges [here](https://challenges.duckietown.org/v4/humans/challenges).


Or, run local evaluation with:

    $ dts challenges evaluate --challenge ![CHALLENGE_NAME]

### Verify the submission(s)

This will make a number of submissions (as described below). You can track the status of these submissions in the command line with:

    $ dts challenges follow --submission ![SUBMISSION_NUMBER]

or through your browser by navigating the webpage: `https://challenges.duckietown.org/v4/humans/submissions/![SUBMISSION_NUMBER]`

where `![SUBMISSION_NUMBER]` should be replaced with the number of the submission which is reported in the terminal output. 

## Anatomy of the submission

The submission consists of all of the basic files that required for a [basic submission](#minimal-template). Below we will highlight the specifics with respect to this template.

### `solution.py`

The only differences in `solution.py` (the python script that is run by our submission) are:

 - We conditionally load the model in the initializaiton procedure:
 
```python
self.model = DDPG(state_dim=self.preprocessor.shape, action_dim=2, max_action=1, net_type="cnn")
self.current_image = np.zeros((640, 480, 3))

if load_model:
    logger.info('PytorchRLTemplateAgent loading models')
    fp = model_path if model_path else "model"
    self.model.load(fp, "models", for_inference=True)
```
 
 -  We abort if no GPU is detected and the environment variable `AIDO_REQUIRE_GPU`.
 
 
 - We are calling our model to compute an action with the following code:

```python
    def compute_action(self, observation):
        action = self.model.predict(observation)
        return action.astype(float)
```

## Model files

The other addition files are the following:

    wrappers.py
    model.py
    models

`wrappers.py` contains a simple wrapper for resizing the input image. `model.py` is used for training the model, and the models are stored in `models`.

