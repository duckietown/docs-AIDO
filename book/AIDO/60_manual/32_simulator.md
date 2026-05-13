# Using Duckiematrix Locally {#aido-simulator status=ready}

The current `ente` learning repositories no longer use the older
`gym-duckietown` server workflow. The maintained local workflow uses a live
Duckiematrix engine running in Gym mode plus a renderer.

This section applies to repositories that build their learning environment
from `gym_duckiematrix.db21j_env.DuckiematrixDB21JEnv` during training or use
`gym_duckiematrix.gym_environment.GymEnvironment` at runtime. In the current
stack that includes the reinforcement-learning baseline, the DAgger baseline,
and custom repositories derived from the PyTorch template.

## What runs where

For local training and debugging, keep these roles separate:

- Your host shell or development container runs the Python trainer and test
    harness.
- A local Duckiematrix engine serves the Gym world and the DTPS endpoint on
    `127.0.0.1:7501`.
- A renderer attaches to that engine and produces the camera stream.
- Local submission validation remains a separate step through
    `dts challenges evaluate`.

The submission image is the artifact you evaluate and submit. It is usually
not the fastest place to iterate on training code.

## DTPS or SHM

The maintained `ente` training environment supports two local transport modes.

### DTPS mode

Leave `DTSHELL_SHM_PATH` unset.

Use DTPS mode when you are bringing a repository up for the first time,
debugging engine startup, or validating that the trainer can reset and step
through the environment at all.

### SHM mode

Set `DTSHELL_SHM_PATH` and start the engine with `--shm-path`.

Use SHM mode when you want the evaluator-style world I/O path or lower
overhead for repeated stepping. SHM only replaces the Gym `WorldInput` and
`WorldOutput` channel. The trainer still needs the DTPS endpoint on
`127.0.0.1:7501` for map data and robot state, so SHM is not a full
replacement for DTPS.

## Local workstation with a display

This is the simplest setup.

1. Confirm that a renderer release is installed locally. If `dts matrix run`
     reports that it cannot find the renderer binary, inspect
     `~/.duckietown/duckiematrix/releases/` and choose an installed version.
2. Start the Duckiematrix engine in Gym mode.
3. Wait until port `7501` is reachable and the renderer has joined.
4. Run a short smoke test before starting a long training job.

Example engine launch:

```bash
dts matrix run --standalone --embedded --map loop \
        --version RENDERER_VERSION \
        --gym --delta-t 0.025 \
        --target-frame-rate -1 \
        --no-pull --profiler
```

Before training, confirm both of these:

- `nc -z 127.0.0.1 7501` succeeds.
- `docker logs dts-matrix-engine` shows `All renderers joined the network`.

Most maintained learning repositories then expose baseline-owned commands such
as:

```bash
python -m training.test
python -m training.train
```

Start with the shortest smoke test the repository supports. Only move to a
longer run after reset, stepping, and checkpoint writing work locally.

## Headless GPU host

The trainer and the renderer have different display requirements.

- The Python environment may still create a Matplotlib figure even if you do
    not call `render()`. On a headless machine, set `MPLBACKEND=Agg` unless you
    intentionally want an interactive backend.
- The Unity renderer still needs a display provider. On a headless Linux
    host, that usually means Xvfb or a dedicated renderer container or sidecar.
- The default training environment is local-only. Unless you have written a
    custom environment, it expects the engine on `127.0.0.1:7501`.

One common host setup is:

```bash
export DISPLAY=:99
export MPLBACKEND=Agg
Xvfb :99 -screen 0 1280x720x24 &
```

If you already run the renderer in a sidecar container, keep the trainer on
the host or in a development container and make sure the engine still appears
locally on `127.0.0.1:7501`.

## Starting in SHM mode

Enable SHM explicitly and keep the DTPS endpoint alive:

```bash
export DTSHELL_SHM_PATH=/tmp/duckiematrix/world_io
mkdir -p /tmp/duckiematrix

dts matrix run --standalone --embedded --map loop \
        --version RENDERER_VERSION \
        --gym --delta-t 0.025 \
        --target-frame-rate -1 \
        --no-pull --profiler \
        --shm-path /tmp/duckiematrix/world_io
```

Before training, confirm all of these:

- the DTPS port `7501` is reachable
- the SHM files appear for the chosen path, typically including `*.e2s` and
    `*.s2e`
- the renderer has joined the engine

Once those conditions are satisfied, the baseline-owned training commands are
the same as in DTPS mode.

## Multi-repository development

If you are co-developing the local simulator stack together with
`duckietown-sdk`, `duckietown-messages`, or `gym-duckiematrix`, prefer
editable installs or a `PYTHONPATH` override so the trainer uses the source
trees you are editing instead of stale installed packages.

Example:

```bash
export PYTHONPATH=/path/to/duckietown-messages/src:/path/to/duckietown-sdk/src:/path/to/gym-duckiematrix/src
```

That is especially useful when you are debugging transport selection,
camera-decoding issues, or changes in the Duckiematrix environment wrapper.

## Recommended workflow

For most repositories, the safest progression is:

1. Bring up the engine and renderer first.
2. Run the repository's shortest local test path.
3. Run a short training job and confirm that checkpoints are written.
4. Re-evaluate the resulting checkpoint locally.
5. Validate the submission container separately with
     `dts challenges evaluate --challenge aido-LF-sim-validation`.

Only after the local runtime and the local trainer both work cleanly is it
worth spending time on longer training runs.

## Common failure modes

- The renderer release named in `dts matrix run` is not installed locally.
- SHM is enabled but port `7501` is still down, so the environment never gets
    map or robot state.
- A headless machine lacks either a display provider for the renderer or
    `MPLBACKEND=Agg` for the trainer.
- Local Python imports still resolve to stale installed packages instead of
    the source trees you are actively editing.
