cmake_minimum_required(VERSION 3.8)
project(tb3_semantic_map_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticEntity.msg"
  "msg/SemanticPlace.msg"
  "msg/SemanticRelation.msg"
  "msg/SemanticAnchor.msg"
  "msg/SemanticMapState.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
