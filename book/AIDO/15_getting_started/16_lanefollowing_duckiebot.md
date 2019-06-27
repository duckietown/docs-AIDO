# Lane Following on Duckiebot {#lf_duckiebot status=ready}

<div class='requirements' markdown='1'>

Requires: You have [built your Duckiebot](+opmanual_duckiebot#assembling-duckiebot-db18)

Requires: You have [built your Duckietown according to the appearance specification](+opmanual_duckietown#dt-ops-appearance-specifications)

Requires: You can [connect to your robot wirelessly](+opmanual_duckiebot#duckiebot-network)

Requires: You have [made a valid submission](#cm-first)

</div>


In this page we will describe how to run your submission on the Duckiebot. If everything's setup right, the procedure is very straightforward. But things can be hard to troubleshoot because they involve networking.

There are two basic modes that you can use to run a submission.

 1. From a local submission folder
 2. From an existing image (for example one that you submitted to the AI-DO)


## Starting the Duckiebot drivers

The first thing to do is to make sure that your camera is streaming imagery and that your robot is accepting wheel commands. If you run:

    $ docker -H ![DUCKIEBOT_NAME].local ps

you should see the list of the running containers. Among them should be one for an image called `duckietown/duckiebot-interface:master19`. If you don't see it at all (weird?) then you can start it with

    $ docker -H ![DUCKIEBOT_NAME].local run --net host --privileged -v /data:/data --name duckiebot-interface duckietown/duckiebot-interface:master19
    

To see what imagery is streaming you can do

    $ dts start_gui_tools ![DUCKIEBOT_NAME]

If this is the first time running the command, you will have to wait a while (up to 10-15 minutes on slow computers) for the docker image to be pulled and see the shell prompt (you might have to press Enter to see the shell prompt). Once you see the shell prompt, type the following to stream the imagery
    
    $ rqt_image_view

## Run a local submission on the Duckiebot

Go into any valid submission folder (i.e. one where you could run `dts submit` and you would make a submission) and run:

    $ dts duckiebot evaluate --duckiebot_name ![DUCKIEBOT_NAME]

## Run an image that's already built on the Duckiebot

    $ dts duckiebot evaluate --duckiebot_name !{DUCKIEBOT_NAME] --image ![IMAGE_NAME]


