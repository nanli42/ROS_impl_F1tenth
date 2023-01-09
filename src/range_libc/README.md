# RangeLibc (ROS version)

This repo is a direct conversion of [range_libc](https://github.com/kctess5/range_libc) in ROS package version with slight modifications in header functions.

It provides ray casting functionality and can be used in ROS C++ nodes by including `range_libc/RangeLib.h`. You might need to adjust your GPU architecture (`-arch=sm_*`) in `CMakeLists.txt`.

Inspired by the fork [droemer7/range_libc](https://github.com/droemer7/range_libc).
