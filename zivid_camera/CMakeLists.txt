cmake_minimum_required(VERSION 3.5)
project(zivid_camera LANGUAGES CXX)

if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
  add_compile_options(-std=c++17)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(COMPILER_WARNINGS
       "Turn on warnings and warnings-as-errors during compilation" OFF)
if(COMPILER_WARNINGS)
  include(CompilerWarnings NO_POLICY_SCOPE)
endif()

function(turn_on_compiler_warnings_if_enabled TARGET)
  if(COMPILER_WARNINGS)
    set_target_warning_compile_options(${TARGET})
  endif()
endfunction()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(zivid_conversions REQUIRED)
find_package(zivid_interfaces REQUIRED)

find_package(Zivid 2.0.0 COMPONENTS Core REQUIRED)
message(STATUS "Found Zivid version ${Zivid_VERSION}")

find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

set(LIBRARY_NAME ${PROJECT_NAME})

add_library(zivid_camera_library SHARED src/zivid_camera.cpp)
turn_on_compiler_warnings_if_enabled(zivid_camera_library)
target_include_directories(zivid_camera_library PRIVATE include)
target_compile_definitions(
  zivid_camera_library PRIVATE
  "ZIVID_ROS_DRIVER_VERSION=\"${${PROJECT_NAME}_VERSION}\"")
target_compile_definitions(zivid_camera_library PRIVATE
                           "ZIVID_CAMERA_BUILDING_DLL")
target_link_libraries(zivid_camera_library Zivid::Core)
ament_target_dependencies(zivid_camera_library
                          rclcpp_lifecycle
                          std_msgs
                          sensor_msgs
                          image_transport
                          zivid_conversions
                          zivid_interfaces)
rclcpp_components_register_node(zivid_camera_library
                                PLUGIN
                                "zivid_camera::ZividCamera"
                                EXECUTABLE
                                "zivid_camera")

add_executable(manual_composition src/manual_composition.cpp)
turn_on_compiler_warnings_if_enabled(manual_composition)
target_include_directories(manual_composition PRIVATE include)
target_link_libraries(manual_composition zivid_camera_library Zivid::Core)
ament_target_dependencies(manual_composition
                          rclcpp_lifecycle
                          std_msgs
                          sensor_msgs
                          image_transport
                          zivid_interfaces)

# since the package installs libraries without exporting them it needs to make
# sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS zivid_camera_library
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(TARGETS manual_composition DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
