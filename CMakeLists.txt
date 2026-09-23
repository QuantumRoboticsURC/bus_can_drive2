cmake_minimum_required(VERSION 3.5)
project(bus_can_drive2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package (Threads REQUIRED)
find_package(SDL2 REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/src/include")
add_executable(drive_can_tank ${PROJECT_SOURCE_DIR}/src/drive_can_tank.cpp)
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCanutils.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCore.a)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_Phoenix.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixCCI.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixCanutils.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixCore.a)
else()
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(drive_can_tank ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCore.a)
    
endif()
target_link_libraries(drive_can_tank Threads::Threads)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#target_link_libraries(intermediate_node Threads::Threads)

ament_target_dependencies(drive_can_tank rclcpp geometry_msgs std_msgs)

add_executable(test_percent_output src/test_percent_output.cpp)

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf")
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCanutils.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCore.a)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu")
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_Phoenix.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixCCI.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixCanutils.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/jetsontx/libCTRE_PhoenixCore.a)
else()
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(test_percent_output ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCore.a)
endif()

target_link_libraries(test_percent_output Threads::Threads)

add_executable(test_current_output src/test_current_output.cpp)
ament_target_dependencies(test_current_output rclcpp std_msgs)
target_link_libraries(test_current_output
  ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_Phoenix.a
  ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCCI.a
  ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixDiagnostics.a
  ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a
  ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCanutils.a
  ${CMAKE_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCore.a
  Threads::Threads
)

install(TARGETS test_current_output
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS test_percent_output
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
drive_can_tank
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
