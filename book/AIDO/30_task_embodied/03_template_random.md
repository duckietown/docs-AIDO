# Random Template for `aido3-LF*` {#minimal-template status=ready}

This section describes the contents of the simplest baseline: a "random" agent. It can be used as a starting point for any of the [`LF`](#lf), [`LFV`](#lf_v), and [`LFVI`](#lf_v_i) challenges.

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission to all of the `LF*` challenges and can view their status and output.

</div>


## Quickstart

### Check out [the repository](https://github.com/duckietown/challenge-aido_LF-template-random):

    $ git clone -b daffy git@github.com:duckietown/challenge-aido_LF-template-random.git


### Change into the directory:

    $ cd challenge-aido_LF-template-random

### Run the submission:

Either make a submission with:

    $ dts challenges submit


Or, run local evaluation with:

    $ dts challenges evaluate

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

This file contains any python requirements that are need your code.

### `solution.py`

The `solution.py` solution file illustrates the protocol interface.

The important parts are:

```python
    def on_received_observations(self,  data: Duckiebot1Observations):
        camera: JPGImage = data.camera
        _rgb = jpg2rgb(camera.jpg_data)
```

which reads an image whenever one becomes available, and

```python
    def on_received_get_commands(self, context: Context):
        if self.n == 0:
            pwm_left = 0.0
            pwm_right = 0.0
        else:
            pwm_left = np.random.uniform(0.5, 1.0)
            pwm_right = np.random.uniform(0.5, 1.0)
        self.n += 1

        # pwm_left = 1.0
        # pwm_right = 1.0
        grey = RGB(0.0, 0.0, 0.0)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = Duckiebot1Commands(pwm_commands, led_commands)
        context.write('commands', commands)
```

which asks for wheel commands to be sent to the robot. Your code must finish by sending the commands to the robot with the `context.write` command.



