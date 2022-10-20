# Minimal pure-Python Template {#minimal-template status=ready}

This section describes the contents of the simplest template: a "random" agent. 

It can be used as a starting point for any of the [`LF`](#challenge-LF), 
[`LFV`](#challenge-LFV), and [`LFI`](#challenge-LFI) challenges.

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission to all of the `LF*` challenges and can view their status and output.

</div>

<figure id="aido-webinar-1">
    <figcaption>Minimal Template</figcaption>
    <dtvideo src="vimeo:477294988"/>
</figure>

## Quickstart

Check out [the repository](https://github.com/duckietown/challenge-aido_LF-template-random):

    $ git clone git@github.com:duckietown/challenge-aido_LF-template-random.git


Change into the directory:

    $ cd challenge-aido_LF-template-random

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

The submission consists of the following files:

    submission.yaml
    Dockerfile
    Makefile
    requirements.txt
    solution.py

### `submission.yaml`

The file `submission.yaml` contains the configuration for this submission:

```
challenge: [c1,c2]
protocol: aido2_db18_agent-z2
user-label: random_agent
user-payload: {}
```

 - With `challenge` you can list [the challenges](#part:aido-rules) that you want your submission to be run on.
 - The `user-label` can be changed to your liking
 - The `protocol` and `user-payload` should probably be left as they are.

### `requirements.txt`

This file contains any python requirements that are needed by your code.


### `solution.py`

The `solution.py` solution file illustrates the protocol interface.

The important parts are:

```python
    def on_received_observations(self, context: Context, data: DB20ObservationsWithTimestamp):
        profiler = context.get_profiler()
        camera: JPGImageWithTimestamp = data.camera
        odometry: DB20OdometryWithTimestamp = data.odometry
        context.info(f"camera timestamp: {camera.timestamp}")
        context.info(f"odometry timestamp: {odometry.timestamp}")
        with profiler.prof("jpg2rgb"):
            _rgb = jpg2rgb(camera.jpg_data)
```

which reads an image whenever one becomes available, and

```python
    def on_received_get_commands(self, context: Context, data: GetCommands):
        self.n += 1

        # behavior = 0 # random trajectory
        behavior = 1  # primary motions

        if behavior == 0:
            pwm_left = np.random.uniform(0.5, 1.0)
            pwm_right = np.random.uniform(0.5, 1.0)
            col = RGB(0.0, 1.0, 1.0)
        elif behavior == 1:
            t = data.at_time
            d = 1.0

            phases = [
                (+1, -1, RGB(1.0, 0.0, 0.0)),
                (-1, +1, RGB(0.0, 1.0, 0.0)),
                (+1, +1, RGB(0.0, 0.0, 1.0)),
                (-1, -1, RGB(1.0, 1.0, 0.0)),
            ]
            phase = int(t / d) % len(phases)
            pwm_right, pwm_left, col = phases[phase]

        else:
            raise ValueError(behavior)

        led_commands = LEDSCommands(col, col, col, col, col)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = DB20Commands(pwm_commands, led_commands)
        context.write("commands", commands)
```

which asks for wheel commands to be sent to the robot. Your code must finish by sending the commands to the robot with the `context.write` command.



