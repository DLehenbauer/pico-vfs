cmake_minimum_required(VERSION 3.13...3.27)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(sd_spi_tests C)
  enable_testing()
endif()

set(CMAKE_C_STANDARD 11)

include(../mocks/fat.cmake)

add_executable(sd_spi_tests
  test_sd_spi.c
  test_sd_fat.c
  ../../src/blockdevice/heap.c
)
target_include_directories(sd_spi_tests PRIVATE mocks)
target_link_libraries(sd_spi_tests PRIVATE fat_test_filesystem)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(sd_spi_tests PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME sd_spi COMMAND sd_spi_tests)
