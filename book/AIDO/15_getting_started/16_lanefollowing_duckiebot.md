# Run an agent on your Duckiebot {#challenge-LF_duckiebot status=ready}

In this page we will describe how to run your submission on your Duckiebot.


<div class='requirements' markdown='1'>

Requires: You have a Duckiebot. See [here](https://www.duckietown.org/about/hardware)
for how to acquire a Duckiebot.

Requires: You have [built your Duckiebot](+opmanual_duckiebot#assembling-duckiebot-db18).

Requires: You have [built your [DB18](+opmanual_duckiebot#assembling-duckiebot-db18) or [DB19](+opmanual_duckiebot#assembling-duckiebot-db19) Duckiebot.

Requires: You have [built your Duckietown according to the appearance specification](+opmanual_duckietown#dt-ops-appearance-specifications).

Requires: You can [connect to your robot wirelessly](+opmanual_duckiebot#duckiebot-network).

Requires: You have [made a valid AI-DO submission](#cm-first).

</div>


Warning: Running your AI-DO submission on your robot is currently only supported on Ubuntu (not Mac OSX).

Warning: If everything's setup right, the procedure is very straightforward. But things can be hard to troubleshoot because they involve networking.

There are two basic modes that you can use to run a submission.

 1. [From a local submission folder](#aido-run-duckiebot-local)
 2. [From an existing image](#aido-run-duckiebot-image) (for example one that you submitted to the AI-DO)

## Verifying that your Duckiebot is operational

When you boot your robot it starts to produce camera imagery and wheel encoder data (if it's moving) and waits for incoming motor commands. To verify that your Duckiebot is fully operational, you should follow [](+opmanual_duckiebot#rc-control) and [](+opmanual_duckiebot#read-camera-data). 

You should also ensure that your Duckiebot is well calibrated, both [camera](+opmanual_duckiebot#camera-calib) and [wheels](+opmanual_duckiebot#wheel-calibration).


## Run a local submission on the Duckiebot {#aido-run-duckiebot-local}

Go into any valid submission folder (i.e. one where you could run `dts submit` and you would make a submission) and run:

    $ dts duckiebot evaluate --duckiebot_name ![DUCKIEBOT_NAME]

## Run an image that's already built on the Duckiebot {#aido-run-duckiebot-image}

    $ dts duckiebot evaluate --duckiebot_name !{DUCKIEBOT_NAME] --image ![IMAGE_NAME]
