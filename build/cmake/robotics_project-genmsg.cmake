# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotics_project: 2 messages, 2 services")

set(MSG_I_FLAGS "-Irobotics_project:/home/leo/UR5-project/src/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotics_project_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPose.msg" NAME_WE)
add_custom_target(_robotics_project_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotics_project" "/home/leo/UR5-project/src/msg/ObjectPose.msg" "geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg" NAME_WE)
add_custom_target(_robotics_project_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotics_project" "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg" "robotics_project/ObjectPose:geometry_msgs/Pose2D"
)

get_filename_component(_filename "/home/leo/UR5-project/src/srv/VisionResults.srv" NAME_WE)
add_custom_target(_robotics_project_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotics_project" "/home/leo/UR5-project/src/srv/VisionResults.srv" "robotics_project/ObjectPose:geometry_msgs/Pose2D:robotics_project/ObjectPoseArray"
)

get_filename_component(_filename "/home/leo/UR5-project/src/srv/SpawnObject.srv" NAME_WE)
add_custom_target(_robotics_project_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotics_project" "/home/leo/UR5-project/src/srv/SpawnObject.srv" "robotics_project/ObjectPose:geometry_msgs/Pose2D"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotics_project
)
_generate_msg_cpp(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotics_project
)

### Generating Services
_generate_srv_cpp(robotics_project
  "/home/leo/UR5-project/src/srv/VisionResults.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotics_project
)
_generate_srv_cpp(robotics_project
  "/home/leo/UR5-project/src/srv/SpawnObject.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotics_project
)

### Generating Module File
_generate_module_cpp(robotics_project
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotics_project
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotics_project_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotics_project_generate_messages robotics_project_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPose.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_cpp _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_cpp _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/VisionResults.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_cpp _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/SpawnObject.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_cpp _robotics_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotics_project_gencpp)
add_dependencies(robotics_project_gencpp robotics_project_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotics_project_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotics_project
)
_generate_msg_eus(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotics_project
)

### Generating Services
_generate_srv_eus(robotics_project
  "/home/leo/UR5-project/src/srv/VisionResults.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotics_project
)
_generate_srv_eus(robotics_project
  "/home/leo/UR5-project/src/srv/SpawnObject.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotics_project
)

### Generating Module File
_generate_module_eus(robotics_project
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotics_project
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotics_project_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotics_project_generate_messages robotics_project_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPose.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_eus _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_eus _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/VisionResults.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_eus _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/SpawnObject.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_eus _robotics_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotics_project_geneus)
add_dependencies(robotics_project_geneus robotics_project_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotics_project_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotics_project
)
_generate_msg_lisp(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotics_project
)

### Generating Services
_generate_srv_lisp(robotics_project
  "/home/leo/UR5-project/src/srv/VisionResults.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotics_project
)
_generate_srv_lisp(robotics_project
  "/home/leo/UR5-project/src/srv/SpawnObject.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotics_project
)

### Generating Module File
_generate_module_lisp(robotics_project
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotics_project
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotics_project_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotics_project_generate_messages robotics_project_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPose.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_lisp _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_lisp _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/VisionResults.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_lisp _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/SpawnObject.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_lisp _robotics_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotics_project_genlisp)
add_dependencies(robotics_project_genlisp robotics_project_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotics_project_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotics_project
)
_generate_msg_nodejs(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotics_project
)

### Generating Services
_generate_srv_nodejs(robotics_project
  "/home/leo/UR5-project/src/srv/VisionResults.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotics_project
)
_generate_srv_nodejs(robotics_project
  "/home/leo/UR5-project/src/srv/SpawnObject.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotics_project
)

### Generating Module File
_generate_module_nodejs(robotics_project
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotics_project
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotics_project_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotics_project_generate_messages robotics_project_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPose.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_nodejs _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_nodejs _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/VisionResults.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_nodejs _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/SpawnObject.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_nodejs _robotics_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotics_project_gennodejs)
add_dependencies(robotics_project_gennodejs robotics_project_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotics_project_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotics_project
)
_generate_msg_py(robotics_project
  "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotics_project
)

### Generating Services
_generate_srv_py(robotics_project
  "/home/leo/UR5-project/src/srv/VisionResults.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/home/leo/UR5-project/src/msg/ObjectPoseArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotics_project
)
_generate_srv_py(robotics_project
  "/home/leo/UR5-project/src/srv/SpawnObject.srv"
  "${MSG_I_FLAGS}"
  "/home/leo/UR5-project/src/msg/ObjectPose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotics_project
)

### Generating Module File
_generate_module_py(robotics_project
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotics_project
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotics_project_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotics_project_generate_messages robotics_project_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPose.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_py _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/msg/ObjectPoseArray.msg" NAME_WE)
add_dependencies(robotics_project_generate_messages_py _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/VisionResults.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_py _robotics_project_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leo/UR5-project/src/srv/SpawnObject.srv" NAME_WE)
add_dependencies(robotics_project_generate_messages_py _robotics_project_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotics_project_genpy)
add_dependencies(robotics_project_genpy robotics_project_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotics_project_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotics_project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotics_project
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotics_project_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robotics_project_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotics_project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotics_project
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotics_project_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(robotics_project_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotics_project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotics_project
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotics_project_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robotics_project_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotics_project)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotics_project
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotics_project_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(robotics_project_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotics_project)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotics_project\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotics_project
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotics_project_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robotics_project_generate_messages_py geometry_msgs_generate_messages_py)
endif()
