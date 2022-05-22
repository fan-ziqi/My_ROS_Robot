cmake_minimum_required(VERSION 2.8.3)
project(world_canvas_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs uuid_msgs geometry_msgs std_srvs message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Annotation.msg
  Annotations.msg
  AnnotationData.msg
  WorldCanvas.msg
  MapListEntry.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DeleteMap.srv
  ListMaps.srv
  PublishMap.srv
  RenameMap.srv
  SaveMap.srv
  DeleteAnnotations.srv
  GetAnnotations.srv
  GetAnnotationsData.srv
  PubAnnotationsData.srv
  EditAnnotationsData.srv
  SaveAnnotationsData.srv
  ListWorlds.srv
  SetKeyword.srv
  SetRelationship.srv
  ResetDatabase.srv
  YAMLImport.srv
  YAMLExport.srv
)

generate_messages(DEPENDENCIES std_msgs uuid_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs uuid_msgs geometry_msgs std_srvs message_runtime)
