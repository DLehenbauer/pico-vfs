add_executable(host
  main.c
  test_blockdevice.c
  test_filesystem.c
  test_benchmark.c
)
target_compile_options(pico_stdio INTERFACE -Wno-error=unused-parameter -Wno-error=sign-compare)
target_compile_options(host PRIVATE
  -Wno-incompatible-function-pointer-types
  -Wno-incompatible-pointer-types
)
target_link_libraries(host PRIVATE
  pico_stdlib
  blockdevice_heap
  filesystem_fat
  filesystem_littlefs
)

if(PICO_PLATFORM STREQUAL "host")
  add_test(NAME host COMMAND host)
  add_subdirectory(../sd_spi sd_spi)
  add_subdirectory(../fat_disk fat_disk)
  add_subdirectory(../vfs vfs)
endif()
