cmake_minimum_required(VERSION 3.13...3.27)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(sd_spi_tests C)
  enable_testing()
endif()

set(CMAKE_C_STANDARD 11)

# The production timeout exercises trim's minimum-budget branch; the short
# timeout alone cannot. That floor is driver policy, not an SD timing minimum.
foreach(target sd_spi_tests sd_spi_tests_production_timeout)
  add_executable(${target} test_sd_spi.c)
  target_include_directories(${target} PRIVATE mocks ../../include)
  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror)
  endif()
endforeach()
target_compile_definitions(sd_spi_tests PRIVATE CONF_SD_CMD_TIMEOUT=5)
add_test(NAME sd_spi COMMAND sd_spi_tests)
add_test(NAME sd_spi_production_timeout COMMAND sd_spi_tests_production_timeout)
