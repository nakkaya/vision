
cmake_minimum_required(VERSION 2.8)
project(vision)

find_package (OpenCV REQUIRED)

add_library(vision SHARED vision.c)
target_link_libraries(vision cxcore cv highgui)
