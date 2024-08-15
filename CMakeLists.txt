# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "components/." "libs/." $ENV{IDF_PATH}/examples/system/console/advanced/components)
# set(EXTRA_COMPONENT_DIRS "libs/.")
# set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/system/console/advanced/components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test-fw)