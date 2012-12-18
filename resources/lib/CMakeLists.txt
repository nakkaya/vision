cmake_minimum_required(VERSION 2.8)
project(vision)

#set(CMAKE_C_FLAGS -m32) 
#LINK_DIRECTORIES( /Homebrew/lib/ )

find_package (OpenCV REQUIRED)

add_executable(calibrate_cam calibrate_cam.c)

IF( OpenCV_VERSION_MINOR GREATER 3 )
  MESSAGE( "Configuring for 2.4" )
  target_link_libraries(calibrate_cam opencv_core
                                      opencv_highgui
                                      opencv_calib3d
                                      opencv_legacy
                                      opencv_imgproc)
ENDIF( OpenCV_VERSION_MINOR GREATER 3 )

IF ( OpenCV_VERSION_MINOR EQUAL 3 )
  MESSAGE( "Configuring for 2.3" )
  target_link_libraries(calibrate_cam opencv_core
                                      opencv_highgui
                                      opencv_calib3d
                                      opencv_imgproc)
ENDIF( OpenCV_VERSION_MINOR EQUAL 3 )

IF ( OpenCV_VERSION_MINOR LESS 2 )
  target_link_libraries(calibrate_cam cv
                                      highgui
                                      cxcore)
ENDIF( OpenCV_VERSION_MINOR LESS 2 )

add_library(vision SHARED vision.c)

if(CYGWIN)
  set_target_properties(vision PROPERTIES LINK_FLAGS "-Wl,--kill-at")
  set(CMAKE_C_FLAGS "-mno-cygwin")
  add_custom_command(TARGET vision POST_BUILD COMMAND
                     "${CMAKE_COMMAND}" -E rename
                     cygvision.dll vision.dll)
endif(CYGWIN)

IF( OpenCV_VERSION_MINOR GREATER 3 )
  MESSAGE("Configuring for 2.4")
  ADD_DEFINITIONS(-DFOO -DCV2_2)
  target_link_libraries(vision opencv_core
                               opencv_highgui
                               opencv_video
                               opencv_calib3d
                               opencv_features2d
                               opencv_flann
                               opencv_imgproc
                               opencv_legacy
                               opencv_objdetect)
ENDIF( OpenCV_VERSION_MINOR GREATER 3 )

IF ( OpenCV_VERSION_MINOR EQUAL 3 )
  MESSAGE("Configuring for 2.3")
  ADD_DEFINITIONS(-DFOO -DCV2_2)
  target_link_libraries(vision opencv_core
                               opencv_highgui
                               opencv_video
                               opencv_calib3d
                               opencv_features2d
                               opencv_flann
                               opencv_imgproc
                               opencv_objdetect)
ENDIF( OpenCV_VERSION_MINOR EQUAL 3 )

IF ( OpenCV_VERSION_MINOR LESS 2 )
  include_directories (/usr/include/opencv/)
  target_link_libraries(vision cv
                               highgui
                               cxcore)
ENDIF( OpenCV_VERSION_MINOR LESS 2 )


find_library(FREENECT_LIBRARY NAMES freenect)

if (FREENECT_LIBRARY)
     MESSAGE("Configuring for Kinect Support")
     ADD_DEFINITIONS(-DFREENECT)
     target_link_libraries(vision freenect_sync)
endif (FREENECT_LIBRARY)
