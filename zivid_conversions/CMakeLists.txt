cmake_minimum_required(VERSION 3.5)
project(zivid_conversions LANGUAGES CXX)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Zivid 2.0.0 COMPONENTS Core REQUIRED)
message(STATUS "Found Zivid version ${Zivid_VERSION}")

find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_library(zivid_conversions src/zivid_conversions.cpp)
turn_on_compiler_warnings_if_enabled(zivid_conversions)
target_include_directories(
  zivid_conversions
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_compile_definitions(zivid_conversions PRIVATE
                           "ZIVID_CONVERSIONS_BUILDING_LIBRARY")
target_link_libraries(zivid_conversions Zivid::Core)
ament_target_dependencies(zivid_conversions sensor_msgs)


install(DIRECTORY include/ DESTINATION include)
install(TARGETS zivid_conversions
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_interfaces(export_${PROJECT_NAME})
ament_export_libraries(zivid_conversions)

ament_package()
