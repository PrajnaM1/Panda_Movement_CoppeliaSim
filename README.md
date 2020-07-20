# Panda_Movement_CoppeliaSim

ARM CLIENT:<br />
Specify type of action and required parameters in this node<br />
<br />
PICK SERVER:<br />
If 'goal.move' is TRUE, 'gripper content' is FALSE and 'object reachability' is TRUE the end effector moves to the 'goal' and then, grasps the object<br />
If 'goal.move' is FALSE, the end effector directly grasps the object<br />
<br />
PLACE SERVER:<br />
If 'goal.move' is TRUE, 'gripper content' is TRUE and 'object reachability' is TRUE, the end effector moves to the 'goal' and then, releases the object<br />
If 'goal.move' is FALSE, the end effector directly releases the object<br />
<br />
GRASP/RELEASE NODE:<br />
Sends the 'grasp'/'release' command through '/panda/commands'<br />
<br />
MOVE TO GOAL SERVER:<br />
Moves end effector to the 'goal' as specified in the client node in interpolated steps<br />
<br />
MOVE TO CONTACT SERVER:<br />
Moves end effector towards a contact in the required direction of force application until the 'force_threshold' is met. <br />
<br />
APPLY FORCE SERVER:<br />
Once the end effector has reached the contact/surface, force is applied on the contact for a time period as specified in the client node. <br />
<br />
SCREW SERVER:<br />
In progress...<br />
<br />
WIPE SERVER:<br />
In progress...<br />
<br />
INSERT SERVER:<br />
In progress<br />
<br />



