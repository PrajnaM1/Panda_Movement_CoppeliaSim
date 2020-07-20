# Panda_Movement_CoppeliaSim
Panda Movement (CoppeliaSim)

ARM CLIENT:
Specify type of action and required parameters in this node

PICK SERVER:
If 'goal.move' is TRUE, the end effector moves to the 'goal' and then, grasps the object
If 'goal.move' is FALSE, the end effector directly grasps the object

PLACE SERVER:
If 'goal.move' is TRUE, the end effector moves to the 'goal' and then, releases the object
If 'goal.move' is FALSE, the end effector directly releases the object

GRASP/RELEASE NODE:
Sends the 'grasp'/'release' command through '/panda/commands'

MOVE TO GOAL SERVER:
Moves end effector to the 'goal' as specified in the client node in interpolated steps

MOVE TO CONTACT SERVER:
Moves end effector towards a contact until the 'force_threshold' is met. 

APPLY FORCE SERVER:
Once the end effector has reached the contact/surface, force is applied on the contact for a time period as specified in the client node. 

SCREW SERVER:
In progress...

WIPE SERVER:
In progress...

INSERT SERVER:
In progress




