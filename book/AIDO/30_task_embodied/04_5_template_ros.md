# ROS Template {#ros-template status=ready}

This section describes the current `ente` procedure for making a lane-following submission with the [Robot Operating System](http://www.ros.org/).

The `challenge-aido_LF-template-ros` repository bridges Duckiematrix world I/O into a small ROS graph and ships a placeholder random-action controller. It is the starting point for the `ente` [`LF`](#challenge-LF) challenge, and the same structure generalizes to other `LF*` challenge definitions that reuse the `aido6_embodied_sys` interface.

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Requires: That you have a basic understanding of [ROS](http://www.ros.org/).

Result: You can build, evaluate, and submit the current ROS template for `aido-LF-sim-validation`.

</div>

## Quickstart 

Clone the [template repo](https://github.com/duckietown/challenge-aido_LF-template-ros):

    $ git clone git@github.com:duckietown/challenge-aido_LF-template-ros.git

Change into the directory:

    $ cd challenge-aido_LF-template-ros

The repository already contains a `submission.yaml` targeting `aido-LF-sim-validation` with protocol `aido6_embodied_sys`.

Make a submission with:

    $ dts challenges submit
    
or run the same image locally with:

    $ dts challenges evaluate --challenge aido-LF-sim-validation

The list of open `ente` challenges is available [here](https://staging-challenges.duckietown.com/humans/challenges).

### Verify the submission:

You can track the status of a submission in the command line with:

    $ dts challenges follow --submission ![SUBMISSION_NUMBER]

or through your browser by navigating to:

    https://staging-challenges.duckietown.com/humans/submissions/![SUBMISSION_NUMBER]

where `![SUBMISSION_NUMBER]` should be replaced with the number of the submission which is reported in the terminal output. 




## Anatomy of the submission

The active `ente` ROS template is organized around these paths:

    assets/calibrations/
    dependencies.txt
    launchers/
    scripts/
    solution/
    submission.yaml

The challenge metadata lives in `submission.yaml`, `dependencies.txt` is the user-editable dependency file, the ROS package lives under `solution/`, the launch entrypoint lives in `launchers/`, and the runtime camera defaults live in `assets/calibrations/`.


### Dockerfile

The `ente` Dockerfile starts from `duckietown/dt-core:BASE_TAG`, installs the runtime Duckietown Python dependencies from `dependencies.txt`, applies the runtime calibration defaults into `/data/config/calibrations`, builds the ROS package under `solution/`, installs the launchers, and finally runs `dt-launcher-LAUNCHER_NAME`.


### `solution/src/main.py`

`solution/src/main.py` is the Duckiematrix-to-ROS bridge. It instantiates `ROSAgent`, receives world input from the evaluator, and republishes the relevant signals into the ROS graph.

The bridge does the following:

- Reads `VEHICLE_NAME` to identify the active vehicle.
- Chooses SHM transport when `DTSHELL_SHM_PATH` is set and otherwise falls back to DTPS.
- Publishes compressed camera images and encoder ticks into ROS.
- Reads wheel commands back from `ROSAgent` and publishes them as `WorldOutput` for the current `session_id`.

The helper `rosagent.py` next to `main.py` handles the ROS-facing details such as image publication, calibration info publication, and wheel-command subscription.

`dependencies.txt` is kept as the user-editable dependency file and now defines the Git-based Duckietown runtime dependencies for the template.

### `solution/src/random_action_node.py`

The template ships a placeholder random-action controller so the repository can build and run end-to-end before you add your own controller. Replace this node, or the launch file that starts it, when turning the template into a real submission.

### `launchers/` {#ros-template-launchers}

The `launchers/run_and_start.sh` script sources the built workspaces, starts `roscore`, launches the placeholder controller with `random_action_node.launch`, and then launches the bridge with `agent_node.launch`. The script ends by copying ROS logs out of the container so the evaluator can expose them as artifacts.

### Runtime contract

During evaluation the runner provides the environment variables that connect the template to the live Duckiematrix session:

- `VEHICLE_NAME`
- `DUCKIEMATRIX_ENGINE_HOSTNAME`
- `DUCKIEMATRIX_ENGINE_PORT`
- `DTSHELL_SHM_PATH` when SHM mode is enabled

Use this template when you want to write your own ROS controller. Use the [Duckietown ROS baseline](#ros-baseline) when you want the stock lane-following stack instead of the placeholder random-action behavior.

