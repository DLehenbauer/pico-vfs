cmake_minimum_required(VERSION 3.13...3.27)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(sd_spi_tests C)
  enable_testing()
endif()

set(CMAKE_C_STANDARD 11)

add_library(sd_spi_test_filesystem STATIC
  ../../src/blockdevice/heap.c
  ../../src/filesystem/fat.c
  ../../vendor/ff15/source/ff.c
  ../../vendor/ff15/source/ffsystem.c
  ../../vendor/ff15/source/ffunicode.c
)
target_include_directories(sd_spi_test_filesystem PRIVATE
  mocks ../../include ../../include/filesystem/ChaN ../../vendor/ff15/source
)

add_executable(sd_spi_tests test_sd_spi.c test_sd_fat.c)
target_include_directories(sd_spi_tests PRIVATE mocks ../../include)
target_link_libraries(sd_spi_tests PRIVATE sd_spi_test_filesystem)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(sd_spi_tests PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME sd_spi COMMAND sd_spi_tests)
