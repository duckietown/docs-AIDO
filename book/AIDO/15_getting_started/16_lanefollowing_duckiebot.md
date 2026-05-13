# Run an agent on your Duckiebot {#challenge-LF_duckiebot status=ready}

In this page we will describe how to run your submission on your Duckiebot.


<div class='requirements' markdown='1'>

Requires: You have a Duckiebot. See [here](https://www.duckietown.org/about/hardware)
for how to acquire a Duckiebot.

Requires: You have built your DB19 or, preferably, DB21 Duckiebot. For assembly and setup details, see the [Duckiebot operations manual](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/index.html). Evaluations will be performed using `DB21` Duckiebots.

Requires: You have built your Duckietown according to the [appearance specifications](https://docs.duckietown.org/daffy/opmanual_duckietown/out/index.html).

Requires: You can connect to your robot wirelessly; the [Duckiebot operations manual](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/index.html) covers the network setup workflow.

Requires: You have [made a valid AI-DO submission](#cm-first).

Result: You have run a submission on your physical Duckiebot.

</div>



<div figure-id="fig:aido-webinar-duckiebot" figure-caption="Running your agent on your Duckiebot tutorial.">
    <dtvideo src="vimeo:479462039" style='width:100%;height:auto'/>
</div>

Warning: Running your AI-DO submission on your robot is currently only supported on Ubuntu (not Mac OSX).

Warning: If everything's setup right, the procedure is very straightforward. But things can be hard to troubleshoot because they involve networking.

There are two basic modes that you can use to run a submission.

 1. [From a local submission folder](#aido-run-duckiebot-local)
 2. [From an existing image](#aido-run-duckiebot-image) (for example one that you submitted to the AI-DO)

## Verifying that your Duckiebot is operational

When you boot your robot it starts to produce camera imagery and wheel encoder data (if it's moving) and waits for incoming motor commands. To verify that your Duckiebot is fully operational, check that remote control and camera streaming work correctly using the [Duckiebot operations manual](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/index.html).

You should also ensure that your Duckiebot is well calibrated, especially the camera and wheels; the [Duckiebot operations manual](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/index.html) covers both calibration procedures.


## Run a local submission on the Duckiebot {#aido-run-duckiebot-local}

Go into any valid submission folder (i.e., one where you could run `dts submit` and you would make a submission) and run:

    $ dts duckiebot evaluate --duckiebot_name ![DUCKIEBOT_NAME]

## Run an image that is already built on the Duckiebot {#aido-run-duckiebot-image}

    $ dts duckiebot evaluate --duckiebot_name !{DUCKIEBOT_NAME] --image ![IMAGE_NAME]


## Local workflow using the Exercises API {#aido-exercises-api}

We have also developed a workflow for submitting exercises
in the [Duckietown MOOC on EdX](https://www.edx.org/course/self-driving-cars-with-duckietown)
that may be useful for your development workflow. Several of the AI-DO templates and baselines are also
valid "exercises" and can therefore follow this workflow. 