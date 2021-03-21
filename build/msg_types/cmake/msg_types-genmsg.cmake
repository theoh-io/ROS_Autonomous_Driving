# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "msg_types: 6 messages, 0 services")

set(MSG_I_FLAGS "-Imsg_types:/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Imsg_types:/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genjava REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(msg_types_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg" NAME_WE)
add_custom_target(_msg_types_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msg_types" "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg" "msg_types/Position:msg_types/PositionArray"
)

get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg" NAME_WE)
add_custom_target(_msg_types_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msg_types" "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg" ""
)

get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg" NAME_WE)
add_custom_target(_msg_types_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msg_types" "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg" "msg_types/Position:msg_types/PositionArray:msg_types/State:msg_types/TrajectoryArray"
)

get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg" NAME_WE)
add_custom_target(_msg_types_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msg_types" "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg" NAME_WE)
add_custom_target(_msg_types_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msg_types" "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg" "msg_types/Position"
)

get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg" NAME_WE)
add_custom_target(_msg_types_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msg_types" "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg" ""
)

#
#  langs = gencpp;geneus;genjava;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types
)
_generate_msg_cpp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types
)
_generate_msg_cpp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types
)
_generate_msg_cpp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types
)
_generate_msg_cpp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types
)
_generate_msg_cpp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types
)

### Generating Services

### Generating Module File
_generate_module_cpp(msg_types
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(msg_types_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(msg_types_generate_messages msg_types_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_cpp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_cpp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_cpp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_cpp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_cpp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_cpp _msg_types_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msg_types_gencpp)
add_dependencies(msg_types_gencpp msg_types_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msg_types_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types
)
_generate_msg_eus(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types
)
_generate_msg_eus(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types
)
_generate_msg_eus(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types
)
_generate_msg_eus(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types
)
_generate_msg_eus(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types
)

### Generating Services

### Generating Module File
_generate_module_eus(msg_types
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(msg_types_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(msg_types_generate_messages msg_types_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_eus _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_eus _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_eus _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_eus _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_eus _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_eus _msg_types_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msg_types_geneus)
add_dependencies(msg_types_geneus msg_types_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msg_types_generate_messages_eus)

### Section generating for lang: genjava
### Generating Messages
_generate_msg_java(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types
)
_generate_msg_java(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types
)
_generate_msg_java(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types
)
_generate_msg_java(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types
)
_generate_msg_java(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types
)
_generate_msg_java(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types
)

### Generating Services

### Generating Module File
_generate_module_java(msg_types
  ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types
  "${ALL_GEN_OUTPUT_FILES_java}"
)

add_custom_target(msg_types_generate_messages_java
  DEPENDS ${ALL_GEN_OUTPUT_FILES_java}
)
add_dependencies(msg_types_generate_messages msg_types_generate_messages_java)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_java _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_java _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_java _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_java _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_java _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_java _msg_types_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msg_types_genjava)
add_dependencies(msg_types_genjava msg_types_generate_messages_java)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msg_types_generate_messages_java)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types
)
_generate_msg_lisp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types
)
_generate_msg_lisp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types
)
_generate_msg_lisp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types
)
_generate_msg_lisp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types
)
_generate_msg_lisp(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types
)

### Generating Services

### Generating Module File
_generate_module_lisp(msg_types
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(msg_types_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(msg_types_generate_messages msg_types_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_lisp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_lisp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_lisp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_lisp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_lisp _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_lisp _msg_types_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msg_types_genlisp)
add_dependencies(msg_types_genlisp msg_types_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msg_types_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types
)
_generate_msg_nodejs(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types
)
_generate_msg_nodejs(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types
)
_generate_msg_nodejs(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types
)
_generate_msg_nodejs(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types
)
_generate_msg_nodejs(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types
)

### Generating Services

### Generating Module File
_generate_module_nodejs(msg_types
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(msg_types_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(msg_types_generate_messages msg_types_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_nodejs _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_nodejs _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_nodejs _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_nodejs _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_nodejs _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_nodejs _msg_types_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msg_types_gennodejs)
add_dependencies(msg_types_gennodejs msg_types_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msg_types_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types
)
_generate_msg_py(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types
)
_generate_msg_py(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg;/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types
)
_generate_msg_py(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types
)
_generate_msg_py(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types
)
_generate_msg_py(msg_types
  "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types
)

### Generating Services

### Generating Module File
_generate_module_py(msg_types
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(msg_types_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(msg_types_generate_messages msg_types_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/TrajectoryArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_py _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/Position.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_py _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/StateArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_py _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/ControlCmd.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_py _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/PositionArray.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_py _msg_types_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cconejob/StudioProjects/Autonomous_driving_pipeline/src/msg_types/msg/State.msg" NAME_WE)
add_dependencies(msg_types_generate_messages_py _msg_types_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msg_types_genpy)
add_dependencies(msg_types_genpy msg_types_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msg_types_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msg_types
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(msg_types_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(msg_types_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET msg_types_generate_messages_cpp)
  add_dependencies(msg_types_generate_messages_cpp msg_types_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msg_types
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(msg_types_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(msg_types_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET msg_types_generate_messages_eus)
  add_dependencies(msg_types_generate_messages_eus msg_types_generate_messages_eus)
endif()

if(genjava_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genjava_INSTALL_DIR}/msg_types
    DESTINATION ${genjava_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_java)
  add_dependencies(msg_types_generate_messages_java std_msgs_generate_messages_java)
endif()
if(TARGET geometry_msgs_generate_messages_java)
  add_dependencies(msg_types_generate_messages_java geometry_msgs_generate_messages_java)
endif()
if(TARGET msg_types_generate_messages_java)
  add_dependencies(msg_types_generate_messages_java msg_types_generate_messages_java)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msg_types
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(msg_types_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(msg_types_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET msg_types_generate_messages_lisp)
  add_dependencies(msg_types_generate_messages_lisp msg_types_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msg_types
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(msg_types_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(msg_types_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET msg_types_generate_messages_nodejs)
  add_dependencies(msg_types_generate_messages_nodejs msg_types_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msg_types
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(msg_types_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(msg_types_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET msg_types_generate_messages_py)
  add_dependencies(msg_types_generate_messages_py msg_types_generate_messages_py)
endif()
