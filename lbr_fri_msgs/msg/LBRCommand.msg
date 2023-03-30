# Message that holds commands that are exposed through the
# Fast Robot Interface (FRI).
#
# Following commands are exposed (refer to KUKA::FRI::LBRCommand in friLBRCommand.h for documentation)
#
#  * joint_position [rad]
#  * wrench [N/Nm]
#  * torque [Nm]

float64[7] joint_position
float64[6] wrench
float64[7] torque
