# xm_robothw

Create Date: 2015.11.1

Authors: Luke Liao, myyerrol

Function: 
xm_robothw is the most important part of the hardware_interface. It is the bridge between the controller and hardware. When it works, it can accept xm_robot's commands from the controller's manager and also send xm_robot's state to controller from hardware_embedded. xm_robothw include joint_limits and transmissions, the transmissions can convert joint's angles to the motor's space, it can be seen as a sort of mapping. Of course, the transmissions from motor to joint's space is also accepted.
