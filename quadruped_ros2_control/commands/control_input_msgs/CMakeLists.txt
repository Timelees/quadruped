cmake_minimum_required(VERSION 3.8)
project(control_input_msgs)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Inputs.msg"
        "msg/LeggedCommand.msg"
        "msg/SimImu.msg"
        "msg/SimJoint.msg"
        "msg/SimBase.msg"
        "msg/StateError.msg"
)

ament_package()
