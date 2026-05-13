# Advanced submission options {#submit-advanced status=ready}

This section describes additional options for the `dts challenges submit`
command.


## `submission.yaml` file {#submission-config}

Each submission directory has a file `submission.yaml` containing the following information:

    protocol: aido6_embodied_sys
    challenge: aido-LF-sim-validation
    user-label: ![optional label]
    user-payload: ![optional user payload]
    
You can override these using the command line, as explained below.

## Specifying the challenge {#submit-advanced-name}

However you can also pass the name as a parameter `--challenge`:

    $ dts challenges submit --challenge ![challenge name]
    
The names of the challenges can be seen [at this page][list-challenges].

[list-challenges]: https://staging-challenges.duckietown.com/humans/challenges

For example, if you would like to submit to the maintained public LF validation challenge, you can do it as:

    $ dts challenges submit --challenge aido-LF-sim-validation

If you override the default values in `submission.yaml`, make sure that the challenge and protocol remain compatible. For the maintained templates and baselines in this workspace, that means keeping `aido6_embodied_sys` together with `aido-LF-sim-validation` unless you are intentionally targeting another compatible evaluator.

    protocol: aido6_embodied_sys
    challenge: aido-LF-sim-validation
    
## Metadata {#submit-advanced-metadata}

You can attach two pieces of metadata to your submission.

1. A human-readable label for your identification.
2. A small JSON payload that describes the details of your submission, such as the parameters that you used for your algorithm.


To specify the label, use the option `--user-label`:

    $ dts challenges submit --user-label "My label"

To specify the payload, use the option `--user-meta` and specify a JSON structure:

    $ dts challenges submit --user-meta '{"param":"1"}
   
   
## Skip Docker cache {#submit-advanced-skip-cache}

Use the option `--no-cache` to avoid using the Docker cache and re-build your containers from scratch:

    $ dts challenges submit --no-cache

