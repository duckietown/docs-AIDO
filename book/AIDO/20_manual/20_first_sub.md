# Your first submission {#cm-first status=ready}

This section describes the steps to make your first submission.

## Checkout the test repo

Checkout this repo:

    $ git clone https://github.com/duckietown/challenge-aido1_luck.git


## Submit 

Go to the submission directory:

    $ cd challenge-aido1_luck/submission-random

Submit using:

    $ dts challenges submit
    
What this does is:

1. Build a Docker container.
2. Push the Docker container.
3. Make contact with the [challenge server][server] to send your submission.

[server]: https://challenges.duckietown.org/v3/

The expected output is something along the lines of:

    Sending build context to Docker daemon  5.632kB
    ...
    ...
    Successfully created submission 23
    
    You can track the progress at: https://challenges.duckietown.org/v3/humans/submissions/23
    
    You can also use the command:
    
       dts challenges follow --submission 23

## Monitor the submission

There are 2 ways to monitor the submission:

The first way is to use the web interface, at the URL indicated; in the example, this is:


> [`https://challenges.duckietown.org/v3/humans/submissions/2`](https://challenges.duckietown.org/v3/humans/submissions/2)

The second way is to use the `dts challenges follow` command:

     $ dts challenges follow --submission ![submission ID]
     
     
## Look at the leaderboard


The leaderboard is available at the URL 

    > [`https://challenges.duckietown.org/v3/humans/challenges/aido1_luck-v3/leaderboard`][leaderboard]
    
    
[leaderboard]: https://challenges.duckietown.org/v3/humans/challenges/aido1_luck-v3/leaderboard


       
      
