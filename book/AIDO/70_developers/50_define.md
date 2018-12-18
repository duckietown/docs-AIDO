# Defining challenges {#define status=draft}

The challenge is structure which gives a user's submission a numerical value, a score, which can then be used to compare with other submissions. The challenge is defined in its own repository. A template of a challenge repository can be generated using the cookiecutter here:.


The main parts of the challenge are: the evaluation folder and the challenge.yaml.

#### challenge.yaml

The challenge.yaml file defines the challenge name, metadata about the challenge \(tags,title to display on the server, which dates the challenge should be accessable, a description of the challenge\). This file also defines the associated score and the steps the challenge runs through. These can be defined as follows:

```
scoring:
  scores:
  - name: score1
    description: temp description

steps:
  step1:
    title: Scoring step
    description: ""

    timeout: 100
    evaluation_parameters:
      services:
        evaluator:
          image: _
          build:
            context: ./evaluation
        solution:
          image: SUBMISSION_CONTAINER

    features_required:
      ram_available_mb: 80
      disk_available_mb: 100
```

The top level, steps, defines that we are going to define the steps of the challenge here. The next level includes the steps themselves. In our example we only have a single step, namely the step which generates the scores for our submission. One level down we define the title, description of the step, the timeout time, the parameters to be used for evaluation and the features required. In the evaluation\_parameters we define the services, the evaluator and the solution. The simplest definition for these, and the ones we will use, are the evaluator from the dockerimage we will define in the evaluation folder and the solution image will be the submission image. Finally the features\_required defines how much ram and disk memory we need for our evaluation \(in mb\).	

The next part is the state-machine that defines how we should move step to step. In our case it is defined as follows.

```
transitions:
  - [START, success, step1]
  - [step1, success, SUCCESS]
  - [step1, failed, FAILED]
  - [step1, error, ERROR]
```

We always start at START. On each row we first define the step we begin at, then we define the action, in our case we can have success \(step is successfull\) which leads to the terminal state of SUCCESS. Similarily for failed and error. 

If you have only one step, as the simple regression test here, then you don't have to change this boilerplate.

#### The evaluation

The evaluation is found in the /evaluation folder. The important, non-boilerplate, file is the eval.py. An example of the content of this file can be found bellow.

```
#!/usr/bin/env python
import logging
import math

from duckietown_challenges import wrap_evaluator, ChallengeEvaluator, InvalidSubmission

logging.basicConfig()
logger = logging.getLogger('evaluator')
logger.setLevel(logging.DEBUG)


class Evaluator(ChallengeEvaluator):

    def prepare(self, cie):
        cie.set_challenge_parameters({'dummy':1})

    def score(self, cie):
        solution_output = cie.get_solution_output_dict()
	'''
	Define here how scoring of a solution is done.
	'''
	temp = solution_output['data']
	score = 2*temp
        cie.set_score('score1', score, 'blurb')


if __name__ == '__main__':
    wrap_evaluator(Evaluator())
```

What we care about here is the prepare and the score functions. In the prepare function we can define the input we want to give the users submission code. The simplest way to do this is to define a dictionary containing fields for all the input data and then pass this dictionary to the cie.set\_challenge\_parameters function. This dictionary can then be accessed in the solution code by calling cie.get\_challenge\_parameters. The second part is the score function. This function takes the contents of you put in the solution\_output\_dict in your submission and uses it to generate a numerical score. The numerical score is then passed to the system by calling cie.set\_score with the corresponding score field (since we only have one score we called this score1 in the challenge.yaml file) along with the actual numerical score we want to give to the submission and a text blurb related to the score.

