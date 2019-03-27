# Advanced submission options {#submit-advanced status=ready}

This section describes additional options for the `dts challenges submit`
command.


## `submission.yaml` file {#submission-config}

Each submission directory has a file `submission.yaml` containing the following information:

    protocol: ![protocol] # do not change
    challenge: ![challenge name]
    user-label: ![optional label]
    user-payload: ![optional user payload]
    
You can override these using the command line, as explained below.

## Specifying the challenge {#submit-advanced-name}

However you can also pass the name as a parameter `--challenge`:

    $ dts challenges submit --challenge ![challenge name]
    
The names of the challenges can be seen [at this page][list-challenges].

[list-challenges]: https://challenges.duckietown.org/v4/humans/challenges

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


## Other options useful for debugging {#submit-advanced-debug}


### Do not submit {#submit-advanced-no-submit}


To only build and push, but not submitting, use the option `--no-submit`:

    $ dts challenges submit --no-submit

### Do not push  {#submit-advanced-no-push}

To only build and not pushing, use the option `--no-push`:

    $ dts challenges submit --no-push

