#!/bin/bash

set -eux

#ARGS="-DCMAKE_BUILD_TYPE=Debug"
ARGS="-DCMAKE_BUILD_TYPE=Release"

rm -rf build devel install &&
VERBOSE=1 catkin_make $ARGS &&
VERBOSE=1 catkin_make $ARGS install
