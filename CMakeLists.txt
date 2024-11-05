# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robot_rest_esp32_cybertech)

# set(OUTPUT_DIR "${CMAKE_SOURCE_DIR}/${CMAKE_PROJECT_NAME}_out")
# file(MAKE_DIRECTORY ${OUTPUT_DIR})

# add_custom_target(copy_binaries ALL
# COMMAND ${CMAKE_COMMAND} -E copy
# "${CMAKE_BINARY_DIR}/bootloader/bootloader.bin"
# "${OUTPUT_DIR}/bootloader.bin"
# COMMAND ${CMAKE_COMMAND} -E copy
# "${CMAKE_BINARY_DIR}/partition_table/partition-table.bin"
# "${OUTPUT_DIR}/partition-table.bin"
# COMMAND ${CMAKE_COMMAND} -E copy
# "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
# "${OUTPUT_DIR}/${CMAKE_PROJECT_NAME}.bin"
# COMMENT "Copying binaries to output directory"
# )

# add_dependencies(copy_binaries ${CMAKE_PROJECT_NAME}.elf)