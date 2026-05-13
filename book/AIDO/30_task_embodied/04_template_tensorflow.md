# TensorFlow Template  {#tensorflow-template status=ready}

This section describes the current `ente` procedure for making a submission with TensorFlow.

The `challenge-aido_LF-template-tensorflow` repository is the active TensorFlow template for `aido-LF-sim-validation`. It talks directly to Duckiematrix through `gym-duckiematrix` and keeps the runtime payload in `solution/` while optional TensorFlow checkpoints live under `models/`.

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You can build, evaluate, and submit the current TensorFlow template for `aido-LF-sim-validation`.

</div>

## Quickstart

Clone the [template repo](https://github.com/duckietown/challenge-aido_LF-template-tensorflow):

	$ git clone git://github.com/duckietown/challenge-aido_LF-template-tensorflow.git

Change into the directory:

	$ cd challenge-aido_LF-template-tensorflow

The repository already includes a `submission.yaml` targeting `aido-LF-sim-validation` with protocol `aido6_embodied_sys`.

Make a submission with:

	$ dts challenges submit

or run the same image locally with:

	$ dts challenges evaluate --challenge aido-LF-sim-validation

The list of open `ente` challenges is available [here](https://staging-challenges.duckietown.com/humans/challenges).

### Verify the submission(s)

You can track the status of a submission in the command line with:

	$ dts challenges follow --submission ![SUBMISSION_NUMBER]

or through your browser by navigating to:

	https://staging-challenges.duckietown.com/humans/submissions/![SUBMISSION_NUMBER]

where `![SUBMISSION_NUMBER]` should be replaced with the number of the submission which is reported in the terminal output.

## Anatomy of the submission

The active `ente` TensorFlow template keeps the repository layout intentionally small:

	config.yaml
	dependencies.txt
	models/
	solution/
	submission.yaml
	training/

`submission.yaml` declares the challenge and protocol, `config.yaml` provides exercise metadata for the Duckietown tooling, `dependencies.txt` is the user-editable dependency file, `solution/` contains the runtime payload, `models/` is where optional TensorFlow checkpoints live, and `training/` is a placeholder scaffold so the template matches the maintained PyTorch template structurally. Unlike the historical template, the maintained `ente` metadata does not request a GPU-specific submission feature flag.

### Dockerfile

The Dockerfile installs all Python dependencies from `dependencies.txt`, including published packages from PyPI and the maintained Duckietown repositories from GitHub, and then copies `solution/`, `training/`, and `models/` into `/workspace`. The runtime entrypoint is:

	$ python3 -m solution.main

### `solution/main.py`

`solution/main.py` uses `gym_duckiematrix.gym_environment.GymEnvironment` to talk to the live evaluator session. At runtime it:

- Reads `DUCKIEMATRIX_ENGINE_HOSTNAME` and `DUCKIEMATRIX_ENGINE_PORT`.
- Resolves the active vehicle name from `VEHICLE_NAME`.
- Decodes the compressed camera image from Duckiematrix.
- Calls the TensorFlow policy from `solution/policy.py`.
- Converts `[velocity, steering]` into left and right wheel PWM commands with `solution/action_wrapper.py`.

## Model files

The runtime-specific directories are:

	config.yaml
	dependencies.txt
	models/
	solution/
	training/

`config.yaml` carries the exercise metadata, `dependencies.txt` is kept as the user-editable dependency file, and `solution/policy.py` returns a stationary action by default and restores a TensorFlow checkpoint from `models/` when one is present using the graph definition in `solution/model.py`. Unlike the ROS repositories, this template does not need `launchers/` or `assets/` because it runs a single Python entrypoint instead of a ROS graph.

Replace the placeholder policy or add TensorFlow checkpoint files under `models/` when turning the template into a real submission.
