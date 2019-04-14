# Transferring a Submission to Real Duckietown {#embodied-transfer_sim_to_real status=ready}


This section describes how to get your submission running on your Duckiebot.

<div class='requirements' markdown='1'>

Requires: That you have [built your Duckiebot and set it up](http://docs.duckietown.org/DT18/opmanual_duckiebot/out/building_duckiebot_c0.html) 

Requires: You are connected to your robot through a 5GHz wifi link (if running "remotely"). 

Result: You can see your submission run on your Duckiebot

</div>

Note: always make sure that you `duckietown-shell` is up to date

## Running an Existing Submission

In this case, you have an existing submission that you have already made (i.e. you can view the submission by navigating to https://challenges.duckietown.org/v4/humans/submissions/`![submission_number]`.

There are basically two simple steps: 1) Find the name of your docker image, 2) run it.

### Find the Image Associated with your Submission

 - Navigate to https://challenges.duckietown.org/v4/humans/submissions/`![submission_number]` where `![submision_number]` is the number of the submission that you would like to test on your robot. 

 - Scroll down and click on the triangle next to `29 artifacts` in the row for `step1-simulation`.

 - Scroll down and click on `docker-compose.yaml`. You should see some text that looks like:
 
```

networks: 
  evaluation: {}
services:
  evaluator:
    environment:
      challenge_name: aido1_LF1_r3-v3
      challenge_step_name: step1-simulation
      launcher_parameters: "challenge: LF\nepisodes: 5\nsteps_per_episode: 500\ninclude_map:\
        \ true\nenvironment-constructor: Simulator\nenvironment-parameters: {seed:\
        \ 123, map_name: loop_empty, max_steps: 500001, domain_rand: 0,\n  camera_width:\
        \ 640, camera_height: 480, accept_start_angle_deg: 6, full_transparency: true,\n\
        \  distortion: true}\nagent-info: {challenge: LF}\n"
      solution_parameters: {env: Duckietown-Lf-Lfv-Navv-Silent-v1}
      uid: 0
      username: root
    image: andreacensi/aido1_lf1_r3-v3-step1-simulation-evaluator:2018_11_22_15_24_19
    networks:
      evaluation:
        aliases:
        - evaluation
    volumes:
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/challenge-solution-output:/challenge-solution-output:rw
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/challenge-results:/challenge-results:rw
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/challenge-description:/challenge-description:rw
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/challenge-evaluation-output:/challenge-evaluation-output:rw
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/previous-steps:/previous-steps:rw
  solution:
    environment:
      challenge_name: aido1_LF1_r3-v3
      challenge_step_name: step1-simulation
      uid: 0
      username: root
    image: ![username]/![challenge-name]-submission:![date]
    networks:
      evaluation:
        aliases:
        - evaluation
    volumes:
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/challenge-solution-output:/challenge-solution-output:rw
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/challenge-results:/challenge-results:rw
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/challenge-description:/challenge-description:rw
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/challenge-evaluation-output:/challenge-evaluation-output:rw
    - /tmp/duckietown/DT18/evaluator/executions/aido1_LF1_r3-v3/submission1633/step1-simulation-idsc-rudolf-18140-job14824/previous-steps:/previous-steps:rw
version: 3.0

```

You need to take note of the name of your image: `![username]/![challenge-name]-submission:![date]` - from now on we will refer to it as `![image-name]`

### Running your Submission

On your laptop run:

    laptop $ dts duckiebot evaluate ![robot-name] --image ![image-name]

Where `![robot-name]` is the name of your robot that you are on the same network as (i.e. you can `ssh` into it and `ping` it) and `![image-name]` is the name of the image that you would like to run (the one you found in the previous section). 

## Running a New Submission Directly on the Duckiebot 

TODO: Liam
