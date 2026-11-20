include_guard(GLOBAL)

add_library(fat_test_filesystem STATIC
  ${CMAKE_CURRENT_LIST_DIR}/../../src/filesystem/fat.c
  ${CMAKE_CURRENT_LIST_DIR}/../../vendor/ff15/source/ff.c
  ${CMAKE_CURRENT_LIST_DIR}/../../vendor/ff15/source/ffsystem.c
  ${CMAKE_CURRENT_LIST_DIR}/../../vendor/ff15/source/ffunicode.c
)
target_include_directories(fat_test_filesystem PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/../../include
  ${CMAKE_CURRENT_LIST_DIR}/../../include/filesystem/ChaN
  ${CMAKE_CURRENT_LIST_DIR}/../../vendor/ff15/source
)
