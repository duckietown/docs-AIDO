# Lane Following Quickstart {#quickstart-lanefollowing}

Maintainer: Liam and Andrea C.

<div class='requirements' markdown='1'>

Requires: You have done the [](#quickstart-preliminaries)

Result: You have made a submission to the Lane Following AI-DO challenge and you know how to try to make it better

</div>


1. Do a submission by following the [random template](#challenge-aido1_lf1-template-random) or the [tensorflow template](#tensorflow-template) ([pytorch template](#baseline-pytorch) and [ROS template](#embodied-classic) coming shortly)

Congrats you have now made a submission, but it probably wasn't very good (unless you got very lucky).


2. Try to make your score go up. Now is when you might want to take a look at [](#part:aido-rules) which describe in detail how your score is generated for the specific challenges. For the lane following challenge,  we are currently offering 4 suggested methods to do this (our baseline templates for these options are at various stages of readiness but will be getting updated very soon):
  1. Use [good old fashioned classical robotics and ROS](#embodied_classic)
  2. Use [reinforement learning](#embodied_rl)
  3. Use [imitation learning from data generated in the simulator](#embodied_il_sim)
  4. Use [imitation learning from data from real robots](#embodied_il_logs)
  
Of course you may also choose to use these methods in combination. 

If you choose on the "Learning-based" methods (i.e. 2-4) then you may want to look at the [](#ml-primer)
