# Agent protocols {#agent-protocols status=ready}

This section describes the submission protocols used by the current `ente` AIDO lane-following stack.

## `aido6_embodied_sys` {#aido6_embodied_sys}

`aido6_embodied_sys` is the active submission protocol for the `ente` lane-following challenge `aido-LF-sim-validation`.

### Execution model

- The evaluator starts a Duckiematrix engine and the user solution container.
- The runner injects `VEHICLE_NAME`, `DUCKIEMATRIX_ENGINE_HOSTNAME`, and `DUCKIEMATRIX_ENGINE_PORT` into the solution environment.
- ROS-based solutions may also receive `DTSHELL_SHM_PATH`; when that variable is set, the ROS template uses shared-memory world I/O instead of DTPS.

### Observations

Observations arrive as `WorldInput` messages keyed by vehicle name and tagged with a `session_id`.

For the active LF challenge, the most important observation is the compressed front camera image for `map_0/vehicle_0`. The ROS bridge also republishes encoder ticks when they are present.

### Actions

Actions are wheel commands for the active vehicle. In the current templates and baselines, those commands are normalized PWM values in the interval `[-1, 1]` for the left and right wheels.

The evaluator advances the Gym-mode Duckiematrix simulation when the solution publishes the next action.

### Reference implementations

- The [PyTorch template](#pytorch-template) talks directly to Duckiematrix through `gym_duckiematrix.gym_environment.GymEnvironment`.
- The [ROS template](#ros-template) bridges the same protocol into ROS topics and publishes `WorldOutput` messages for the current `session_id`.

## Historical note {#aido2_db18_agent-z2}

Older AIDO2 and AIDO5 pages refer to the historical `aido2_db18_agent-z2` protocol. That is not the active submission path on `ente`; use [`aido6_embodied_sys`](#aido6_embodied_sys) for the current lane-following stack.
 
