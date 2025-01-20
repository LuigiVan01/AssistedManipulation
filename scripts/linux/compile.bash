#!/bin/bash
# Calls the compiler for compiling and executable creation
cmake --build build
# Set up an organized  folder of the project in the folder install 
cmake --install build --prefix install