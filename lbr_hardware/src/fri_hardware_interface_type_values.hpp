#pragma once

// see https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp
namespace LBR {
    constexpr char HW_IF_EXTERNAL_TORQUE[] = "external_torque";
    constexpr char HW_IF_SAMPLE_TIME[] = "sample_time";
    constexpr char HW_IF_TIME_STAMP_SEC[] = "time_stamp_sec";
    constexpr char HW_IF_TIME_STAMP_NANO_SEC[] = "time_stamp_nano_sec";
}  // end of namespace LBR
