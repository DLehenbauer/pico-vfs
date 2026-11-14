cmake_minimum_required(VERSION 3.13...3.27)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(sd_spi_tests C)
  enable_testing()
endif()

set(CMAKE_C_STANDARD 11)

add_executable(sd_spi_tests test_sd_spi.c)
target_include_directories(sd_spi_tests PRIVATE mocks ../../include)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(sd_spi_tests PRIVATE -Wall -Wextra -Werror)
endif()

foreach(case singleblock start-rejected token-busy-wrap token-timeout-wrap wide-busy-timeout)
  add_test(NAME sd_spi.${case} COMMAND sd_spi_tests ${case})
endforeach()
