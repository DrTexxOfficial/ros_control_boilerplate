include_directories(
  include/
)

# Scorbot Hardware Interface
add_library(scorbot_hw_interface
  src/scorbot_hw_interface.cpp
)
target_link_libraries(scorbot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(scorbot_hw_main src/scorbot_hw_main.cpp)
target_link_libraries(scorbot_hw_main
  scorbot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    scorbot_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    scorbot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/scorbot_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
