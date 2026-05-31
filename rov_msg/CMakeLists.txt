cmake_minimum_required(VERSION 3.8) # 建议提升到3.8以获得更好的Action支持
project(rov_msg)

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED) # Action必须依赖此包

# 声明你的接口文件路径
set(msg_files
  "msg/PidMsg.msg"
  "msg/Imu.msg"
  "msg/Dvl.msg"
  "msg/Controller.msg"
)

set(srv_files
  "srv/Circle.srv"
  "srv/Cruise.srv"
)

set(action_files
  "action/MoveTo.action"
)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs
)

# 导出运行时依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()