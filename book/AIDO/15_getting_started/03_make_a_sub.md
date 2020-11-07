# Make your first submission {#cm-first status=ready}

This section describes the steps to make your first submission.

<div class='requirements' markdown='1'>

Requires: You have [set up your accounts](#cm-accounts).

Requires: You have [the software requirement](#cm-sw).

Result: You have made a submission to the Lane Following AI-DO challenge, and you know how to try to make it better.

</div>

## Checkout the submission repo {#cm-first-checkout}

Check out the competition template [`challenge-aido_LF-template-random`][template]:

    $ git clone https://github.com/duckietown/challenge-aido_LF-template-random

[template]: https://github.com/duckietown/challenge-aido_LF-template-random

## Submit {#cm-first-submit}

Jump into the directory:

    $ cd challenge-aido_LF-template-random

Submit using:

    $ dts challenges submit --challenge aido5-LF-sim-validation

<img class="screencast" src="rec-submit.gif" width="100%"/>



<style>
@media print {
  .screencast {
    display: none;
  }
}
</style>

What this does is:

1. Build a Docker container.
2. Push the Docker container.
3. Make contact with the [challenge server][server] to send your submission.

[server]: https://challenges.duckietown.org/v4/

The expected output is something along the lines of:

    Sending build context to Docker daemon  5.632kB
    ...
    ...
    Successfully created submission ![SUBMISSION_NUMBER]
    
    You can track the progress at: https://challenges.duckietown.org/v4/humans/submissions/![SUBMISSION_NUMBER]
    
    You can also use the command:
    
       dts challenges follow --submission ![SUBMISSION_NUMBER]

where `![SUBMISSION_NUMBER]` is your submission id.

## Monitor the submission {#cm-first-monitor}

There are 2 ways to monitor the submission:

The first way is to use the web interface, at the URL indicated.

The second way is to use the `dts challenges follow` command:

     $ dts challenges follow --submission ![SUBMISSION_NUMBER]

## Look at the leaderboard {#cm-first-leaderboard}

The leaderboard for this challenge is available at the URL

> [`https://challenges-stage.duckietown.org/humans/challenges/aido5-LF-sim-validation/leaderboard`][leaderboard]


[leaderboard]: https://challenges-stage.duckietown.org/humans/challenges/aido5-LF-sim-validation/leaderboard

In general all of the challenge leader boards can be viewed at the front page [the challenges website][challenges].

[challenges]: https://challenges.duckietown.org/v4/



## Local evaluation {#cm-local}

You can also evaluate the submission *locally*.  This is useful for debugging and development.

Use this command:

    $ dts challenges evaluate  --challenge aido5-LF-sim-validation

## Troubleshooting

If any of the commands above don't work, it is likely that something related to Docker permissions is to blame - please file an issue, as we are trying to fix that problem.

<!-- TODO: where to file an issue? -->


<!-- 

###  Look at the other strawman solutions

* [TensorFlow template](#tensorflow-template) 
* [PyTorch template](#pytorch-template) 
* [ROS template](#ros-template) 

### Try to make your score go up

Now is when you might want to take a look at [](#part:aido-rules) which describe in detail how your score is generated for the specific challenges. For the lane following challenge,  we are currently offering 4 suggested methods to do this (our baseline templates for these options are at various stages of readiness but will be getting updated very soon):


* Use ["classical" robotics and ROS](#ros-baseline)
* more to come...
  
Of course you may also choose to use these methods in combination. 

### Try one of the harder challenges 

Like [LFP](#challenge-LFP) or [LFV_multi](#challenge-LFV_multi) or run your submission [on your Duckiebot](#challenge-LF_duckiebot).

-->