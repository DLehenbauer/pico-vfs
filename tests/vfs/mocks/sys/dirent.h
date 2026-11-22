#pragma once

#include "filesystem/filesystem.h"

// The firmware's directory API uses fs_dir_t, not the host libc's opaque DIR.
typedef fs_dir_t DIR;

DIR *opendir(const char *path);
int closedir(DIR *dir);
struct dirent *readdir(DIR *dir);
