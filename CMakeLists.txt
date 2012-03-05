
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ranger-segmenter)

include(UsePlayerC++)
include_directories(${PLAYERCPP_INCLUDE_DIRS})

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(ranger-segmenter ranger-segmenter.cpp)

target_link_libraries(ranger-segmenter
    ${PLAYERCPP_LINK_LIBS}
    ${ARMADILLO_LIBRARIES}
)
