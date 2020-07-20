# Panda_Movement_CoppeliaSim

ARM CLIENT:<br />
Specify type of action and required parameters in this node<br />
<br />
PICK SERVER:_
If 'goal.move' is TRUE, the end effector moves to the 'goal' and then, grasps the object_
If 'goal.move' is FALSE, the end effector directly grasps the object_
_
PLACE SERVER:_
If 'goal.move' is TRUE, the end effector moves to the 'goal' and then, releases the object_
If 'goal.move' is FALSE, the end effector directly releases the object_
_
GRASP/RELEASE NODE:_
Sends the 'grasp'/'release' command through '/panda/commands'_
_
MOVE TO GOAL SERVER:_
Moves end effector to the 'goal' as specified in the client node in interpolated steps_
_
MOVE TO CONTACT SERVER:_
Moves end effector towards a contact until the 'force_threshold' is met. _
_
APPLY FORCE SERVER:_
Once the end effector has reached the contact/surface, force is applied on the contact for a time period as specified in the client node. _
_
SCREW SERVER:_
In progress..._
_
WIPE SERVER:_
In progress..._
_
INSERT SERVER:_
In progress_
_



