# This is a message that holds states that are exposed through the
# Fast Robot Interface (FRI).
#
# Following states are exposes (also refer to KUKA::FRI::LBRState in friLBRState.h)
#
#  * sample_time [s]: This is the period in which the KUKA Sunrise controller is sending FRI packets.
#  * time_stamp_sec [s]: The timestamp of the current robot state in Unix time.
#                        This method returns the seconds since 0:00, January 1st, 1970 (UTC).
#                        Use getTimestampNanoSec() to increase your timestamp resolution when
#                        seconds are insufficient.
#  * time_stampe_nano_sec [ns]: The nanoseconds elapsed since the last second (in Unix time).
#                               This method complements getTimestampSec() to get a high resolution timestamp.
#  * name: Joint name.
#  * position [rad]: The currently measured joint positions of the robot.
#  * torque [Nm]: The currently measured joint torques of the robot.
#  * external_torque [Nm]: The currently measured external joint torques of the robot.
#                          The external torques corresponds to the measured torques when removing
#                          the torques induced by the robot itself. 

float64 sample_time
uint32 time_stamp_sec
uint32 time_stamp_nano_sec
uint32 e_client_command_mode

float64[] measured_joint_position
float64[] ipo_joint_position
float64[] torque
float64[] external_torque
