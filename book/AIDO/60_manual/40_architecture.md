# Evaluation architecture {#arch status=beta}

This section explains what happens behind the scenes when you create a submission on the current `ente` lane-following stack.

## Actors

We have the following actors:

* Your *host computer*, where you initiate the submission.
* The *Challenges Server*, currently exposed publicly at [https://staging-challenges.duckietown.com](https://staging-challenges.duckietown.com).
* An *evaluator runner*, which can run online or locally.
* The *solution container* that you build from your submission repository.
* The *Duckiematrix evaluator stack*, which runs the simulation engine and renderer sidecar.

## Steps

### Building

* You run `dts challenges submit` on the host computer.
* The `dts` command looks for a file called `submission.yaml` in the current directory.
* `submission.yaml` declares the challenge name, protocol, and optional user label and payload.
* The solution image is built from the local `Dockerfile`.
* The image is pushed to a container registry and identified by an immutable digest before the submission is registered.

### Submission

* The host computer connects to the Challenges Server and proposes the submission.
* The information passed includes the image digest, the challenge name, the protocol, the optional user label, and the optional JSON payload.
* The server checks that the challenge exists, the declared protocol is compatible, and your token is valid.

### Waiting

* At this point, the user waits, optionally by looking at the output `dts challenges follow --submission ![ID]`.
* Alternatively, the user can look at the website for updates.

### Execution

* The submission becomes available for execution.
* The server computes which evaluation steps need to run.
* Evaluator runners periodically contact the server and advertise their available features.
* If a job is available, it is assigned to a compatible evaluator.
* The evaluator pulls the submission image and the evaluation image defined by the challenge. For `aido-LF-sim-validation`, that evaluator image is `duckietown/dt-duckiematrix:ente-amd64`.
* The evaluator starts the submission and evaluator containers together.
* The evaluator injects the live Duckiematrix connection information into the solution environment:

  * `VEHICLE_NAME`
  * `DUCKIEMATRIX_ENGINE_HOSTNAME=evaluator`
  * `DUCKIEMATRIX_ENGINE_PORT=7501`
  * `DTSHELL_SHM_PATH=/fifos/world_io` when SHM mode is enabled

* Duckiematrix runs the LF loop-map evaluation in Gym mode while the renderer sidecar handles rendering.
* The evaluator stores logs, frames, and `challenge_results.yaml` as artifacts.
* The evaluator reports the result to the server as `success`, `failed`, or `error`.

## Local evaluation

Local evaluation uses the same submission image and the same challenge definition, but against a local evaluator instead of the public queue:

  dts challenges evaluate --challenge aido-LF-sim-validation

That is the fastest way to debug launcher issues, calibration issues, and baseline changes before creating a public submission.

## Evaluation features {#evaluation-features}

Some submissions or evaluation containers require special features.

You can inspect these features on an evaluator description page such as [this one](https://staging-challenges.duckietown.com/humans/evaluators/1).

These include:

* Memory and disk available,
* CPU architecture and speed,
* GPU available

and others.

Most features are auto-detected by the evaluator. Some evaluator tooling also exposes a `--features` option for overriding detected values when that is required.

