cmake_minimum_required(VERSION 2.8.0)
project(press_machine_IMC)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  message_generation
  message_runtime
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package()

#############
## Install ##
#############
catkin_install_python(
  PROGRAMS
  scripts/heatpress_operator_node.py
  scripts/coldpress_operator_node.py
  scripts/cutter_operator_node.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
