cmake_minimum_required(VERSION 3.14)
project(PathPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(planner src/AStar.cpp src/Dijkstra.cpp)
target_include_directories(planner PUBLIC include)

add_executable(planner_demo src/main.cpp)
target_link_libraries(planner_demo planner)

# Warning flags: compiler-specific, and only on OUR targets (not googletest)
function(set_project_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()
set_project_warnings(planner)
set_project_warnings(planner_demo)

# Google Test (pehli baar build pe internet chahiye)
include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
add_executable(planner_tests tests/test_planner.cpp)
target_link_libraries(planner_tests planner GTest::gtest_main)
set_project_warnings(planner_tests)
include(GoogleTest)
gtest_discover_tests(planner_tests)
