cmake_minimum_required(VERSION 2.8)
project(vision)

find_package (OpenCV REQUIRED)

add_executable(calibrate_cam calibrate_cam.c)
target_link_libraries(calibrate_cam opencv_core
                                    opencv_highgui
                                    opencv_calib3d
                                    opencv_imgproc)

add_library(vision SHARED vision.c)

if(CYGWIN)
  set_target_properties(vision PROPERTIES LINK_FLAGS "-Wl,--kill-at")
  set(CMAKE_C_FLAGS "-mno-cygwin")
  add_custom_command(TARGET vision POST_BUILD COMMAND
                     "${CMAKE_COMMAND}" -E rename
                     cygvision.dll vision.dll)
endif(CYGWIN)

target_link_libraries(vision opencv_core
                             opencv_highgui
                             opencv_video
                             opencv_calib3d
                             opencv_features2d
                             opencv_flann
                             opencv_imgproc
                             opencv_objdetect)
