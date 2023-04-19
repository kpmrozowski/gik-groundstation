#!/bin/bash
. /opt/ros/noetic/setup.bash
# merge all compile_commands.json recursively into one (for intellisense)
jq -s 'map(.[])' build/**/compile_commands.json >> compile_commands.json
# catkin_make -k0 -j2 -l2 --use-ninja -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 -DCMAKE_EXPORT_COMPILE_COMMANDS=1
catkin_make -k0 --use-ninja -DCMAKE_C_COMPILER=/usr/bin/gcc-9 -DCMAKE_CXX_COMPILER=/usr/bin/g++-9 -DCMAKE_EXPORT_COMPILE_COMMANDS=1
