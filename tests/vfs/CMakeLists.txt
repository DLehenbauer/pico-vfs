cmake_minimum_required(VERSION 3.13...3.27)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(vfs_tests C)
  enable_testing()
endif()

set(CMAKE_C_STANDARD 11)
include(../mocks/fat.cmake)

add_library(vfs_test_filesystem STATIC
  ../../src/filesystem/vfs.c
  ../../src/filesystem/littlefs.c
  ../../vendor/littlefs/lfs.c
  ../../vendor/littlefs/lfs_util.c
)
target_include_directories(vfs_test_filesystem PUBLIC
  mocks ../../vendor/littlefs
)
target_link_libraries(vfs_test_filesystem PUBLIC fat_test_filesystem)

add_executable(vfs_tests
  test_vfs.c ../../src/blockdevice/heap.c ../../src/blockdevice/loopback.c
)
target_link_libraries(vfs_tests PRIVATE vfs_test_filesystem)
target_link_options(vfs_tests PRIVATE
  -Wl,--wrap=open -Wl,--wrap=read -Wl,--wrap=write
  -Wl,--wrap=close -Wl,--wrap=lseek -Wl,--wrap=unlink
)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vfs_tests PRIVATE -Wall -Wextra -Werror)
endif()

add_test(NAME vfs COMMAND vfs_tests)
