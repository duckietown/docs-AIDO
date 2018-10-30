# Lane Following Quickstart {#quickstart-lanefollowing status=ready}

Maintainer: Liam and Andrea C.

<div class='requirements' markdown='1'>

Requires: You have done the [](#quickstart-preliminaries)

Result: You have made a submission to the Lane Following AI-DO challenge and you know how to try to make it better

</div>


1) Do a submission by following one of the templates:
  * [Random template](#challenge-aido1_lf1-template-random)
  * [Tensorflow template](#tensorflow-template) 
  * [PyTorch template](#pytorch-template) 
  * [ROS template](#ros-template) 

Congrats! you have now made a submission, but it probably wasn't very good (unless you got very lucky).

2) Try to make your score go up. Now is when you might want to take a look at [](#part:aido-rules) which describe in detail how your score is generated for the specific challenges. For the lane following challenge,  we are currently offering 4 suggested methods to do this (our baseline templates for these options are at various stages of readiness but will be getting updated very soon):
  * Use [good old fashioned classical robotics and ROS](#embodied_classic)
  * Use reinforement learning (coming soon)
  * Use [imitation learning from data generated in the simulator](#embodied_il_sim)
  * Use [imitation learning from data from real robots](#embodied_il_logs)
  
Of course you may also choose to use these methods in combination. 

If you choose on the "Learning-based" methods (i.e. 2-4) then you may want to look at the [](#ml-primer)
