cmake_minimum_required(VERSION 3.13...3.27)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(fat_disk_tests C)
  enable_testing()
endif()

set(CMAKE_C_STANDARD 11)

include(../mocks/fat.cmake)

add_executable(fat_disk_tests test_fat_disk.c ../../src/blockdevice/heap.c)
target_link_libraries(fat_disk_tests PRIVATE fat_test_filesystem)
target_link_options(fat_disk_tests PRIVATE
  -Wl,--wrap=f_stat -Wl,--wrap=f_opendir -Wl,--wrap=f_closedir
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(fat_disk_tests PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME fat_disk COMMAND fat_disk_tests)
