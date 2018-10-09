# Using the Evaluator {#evaluator status=ready}

This section describes how to use the Challenges evaluators.


## Evaluators {#evaluator-intro}

An evaluator is a machine that is in charge of evaluating the protocols.

You can see the [list of connected evaluators here][list].

[list]: https://challenges.duckietown.org/v3/humans/evaluators

## Running your own evaluator {#evaluator-run}

We have several evaluators online that process jobs.

If you want to avoid being in the queue, you can run your own evaluator.

The command line is:

    $ dts challenges evaluator
    
This evaluator will connect to the server and execute preferentially your submission.

