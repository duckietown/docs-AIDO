# CLI reference  {#cli status=ready}

This section is a reference for how to interact with the challenges server with the command line.


## Account info {#cli-info}

Use this command to see the status of your account:

    $ dts challenges info
    

## Submitting a submission {#cli-submit}
 
The `submit` command allows you to see all of your submissions:

    $ dts challenges submit
    
There are many options for this command, explained in [](#submit-advanced).
    
    
## List submissions {#cli-list}

The `list` command allows you to see all of your submissions:

    $ dts challenges list
    
## Reset a submission {#cli-reset}

*Resetting a submission* means that you discard the evaluations 
already perfomed and you force them to be done again.

    $ dts challenges reset --submission ![ID] 

## Retire a submission {#cli-retire}

*Retiring a submission* means that you declare the submission void.
It will not be evaluated and previous results will be discarded.

    $ dts challenges retire --submission ![ID]

## Follow the fate of a submission {#cli-follow}

The `follow` command polls the server to see whether there are updated:

    $ dts challenges follow --submission ![ID]

## Defining a challenge {#cli-define}

The `define` command allows to *define* a challenge:

    $ dts challenges define
    
This command is not available for regular participants;
however the mechanics of this is explained in [](#define) as it might be helpful to 
debug problems.
