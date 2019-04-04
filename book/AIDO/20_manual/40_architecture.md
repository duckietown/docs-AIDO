# Evaluation architecture {#arch status=ready}

This section explains exactly what is going on behind the scenes when you create a submission.


## Actors 

We have the following actors:

* Your *host computer*, where you initiate the submission.
* The *Challenges Server*, currently available at [`https://challenges.duckietown.org`](http://challenges.duckietown.org).
* An *evaluator*; many are available online, but you can run one also on your host computer. 

## Steps

### Building

* You run `dts challenges build` on the host computer.
* The `challenges build` command of the Duckietown Shell `dts` looks for a file called `submission.yaml` that contains the name of the challenge to submit to.
* The container is built using the `Dockerfile` in the current directory.
* The container is pushed to DockerHub under your account, in a repo called `![username]/![challenge]-solution`.

### Submission

* The host computer connects to the Challenges Server using REST and proposes the submission. The information passed includes the label of the container, the challenge name and protocol, and an optional user label and JSON payload.
* The Challenges Server checks that the challenge exists, the protocol declared is compatible, and that you have a valid token.

### Waiting

* At this point, the user waits, optionally by looking at the output `dts challenges follow --submission ![ID]`.
* Alternatively, the user can look at the website for updates.

### Execution

* The submission becomes available for execution.
* The server computes which steps need to be done (each challenge might have multiple steps with 
a finite-state machine mechanism).
* Evaluators periodically contact the server.
* The server checks if the evaluator has the [features requested](#evaluation-features). 
* If a job is available it is assigned to the evaluator.
* The evaluator pulls all the containers involved - the evaluation and the submission.
* The evaluator downloads from S3 artefacts from previous evaluation steps.
* The evaluator runs them together using Docker Compose.
* Artefacts are uploaded to S3.
* The evaluator reports the results to the server; either `success`, `failed` or `error` (evaluation error).  



## Evaluation features {#evaluation-features}

Some submissions or evaluation containers require special features.

You can see the value of these features in [the description page of an evaluator](https://challenges.duckietown.org/v4/humans/evaluators/1).

These include:

* memory and disk available;
* CPU architecture and speed;
* GPU available;
* ...


Most features are auto-detected by the evaluator. A user can force values for the features [using the `--features` command](#evaluator-advanced-features).

