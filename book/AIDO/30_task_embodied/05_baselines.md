# Baselines {#embodied-baselines status=ready}

First thing first, we invite you to run our baselines.

## Lane Following Baseline

We have prepared for you several baselines you can try.

## Learning with Tensorflow

Describe learning with tensorflow here.

### Learning

### Submitting
Submitting an entry for the Lane Following challenge is a very straighforward process.
First you need to fork the template repo [link](https://github.com/duckietown/challenge-aido1_LF1-template-tensorflow).
Then, there are only three steps you need to follow, namely: *Verify*, *Submit* and *Track*.

### Verify
Firstly, we move to the `submission/` directory and we check our code is ready for submission.
This step will help you to avoid waisting time pushing the image, so first check your entry builds:

``dts challenges submit --no-submit --challenge aido1-dummy_sim-v3``

Wait for the image to build. If everything went well, the last line of the output would look a lot like the following:

```
2018_10_09_11_44_25: digest: sha256:3435701bd31abc2f94d9645acce46540a724daf2453652460ff031206b5c649d size: 5559
```

This means your submission is ready to be uploaded to the server.

### Submit
You are ready to submit the image to the challenges server. 
Simply remove the `--no-submit` flag from the command above or type:

`dts challenges submit --challenge aido1-dummy_sim-v3`

This is going to happen a bit faster now as the image was already built on the *Verify* step.
At the end of the command, you should see something like:

```
2018_10_09_11_50_28: digest: sha256:3435701bd31abc2f94d9645acce46540a724daf2453652460ff031206b5c649d size: 5559

Successfully created submission SUBMISSION_ID

You can track the progress at: https://challenges.duckietown.org/v3/humans/submissions/SUBMISSION_ID
```

### Track
There are two options for you to track the status of your submission.
One is the link provided above that will take you to the Challenges Dashboard (yes, you can snoop others submissions and the leaderboard there) or you can get live updates from the comfort of your terminal by running:

```dts challenges follow --submission SUBMISSION_ID```

At some point, depending on the load of our servers, you will see a message like this:

```

2018-10-09T12:12:32.047985 Complete: True  Status: success  Steps: {u'step1': u'success'}

```
This means your submission has been evaluated.
You can now check the score of the submission and its position on the leaderboard via the Challenges Dashboard. 

Are you on the top? Awesome! No? Well, keep trying... we are sure you will be!

