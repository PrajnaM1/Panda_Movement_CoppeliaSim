# Panda_Movement_CoppeliaSim

ARM CLIENT:<br />
Specify type of action and required parameters in this node<br />
<br />
PICK:<br />
If 'gripper content' is FALSE and 'object reachability' is TRUE, the end effector moves to the 'goal' and then, grasps the object<br />
<br />
PLACE:<br />
If 'gripper content' is TRUE and 'object reachability' is TRUE, the end effector moves to the 'goal' and then, releases the object<br />
<br />
GRASP/RELEASE:<br />
Sends the 'grasp'/'release' command through '/panda/commands'<br />
<br />
MOVE TO GOAL PRIMITIVE:<br />
Moves end effector to the 'goal' as specified in the client node in interpolated steps<br />
<br />
MOVE TO CONTACT PRIMITIVE:<br />
Moves end effector towards a contact in the required direction of force application until the 'force_threshold' is met. <br />
<br />
APPLY FORCE:<br />
Once the end effector has reached the contact/surface, force is applied on the contact as specified in the client node. <br />
<br />
SCREW (In Progress):<br />
Moves end effector to 'goal' and begins the action based on screw depth, screw lead, screw angle, screw direction and screw force. <br />
<br />
WIPE SERVER:<br />
In progress...<br />
<br />
INSERT SERVER:<br />
In progress<br />
<br />



