cmake_minimum_required(VERSION 3.14)
project(bcd_planner)


set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories(OpenCV_INCLUDE_DIRS)
include_directories(/usr/include/eigen3)


set(BCD_PLANNER_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(BCD_PLANNER_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src) 

include_directories(
    ${BCD_PLANNER_INCLUDE_DIR}
)

message(${BCD_PLANNER_INCLUDE_DIR})
message(${BCD_PLANNER_SOURCE_DIR})
message(${PROJECT_NAME})


add_library(${PROJECT_NAME} SHARED
    src/bcd_planner.cpp
    src/bcd_planner_wrapper.cpp
)

target_link_libraries(${PROJECT_NAME}
)

add_executable(test_node
    src/test.cpp
)
target_link_libraries(test_node
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)
