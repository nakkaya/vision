cmake_minimum_required(VERSION 2.8)
project(vision)

#set(CMAKE_C_FLAGS -m32)
#INCLUDE_DIRECTORIES(/Homebrew/Cellar/opencv/2.2/include/)
#INCLUDE_DIRECTORIES(/Homebrew/Cellar/opencv/2.2/include/opencv/)
#INCLUDE_DIRECTORIES(/Homebrew/Cellar/opencv/2.2/include/opencv2/)
#LINK_DIRECTORIES(/Homebrew/Cellar/opencv/2.2/lib/)

find_package (OpenCV REQUIRED)

add_executable(calibrate_cam calibrate_cam.c)
target_link_libraries(calibrate_cam opencv_core
                                    opencv_highgui
                                    opencv_calib3d
                                    opencv_imgproc)

add_library(vision SHARED vision.c)
target_link_libraries(vision opencv_core
                             opencv_highgui
                             opencv_video
                             opencv_calib3d
                             opencv_features2d
                             opencv_flann
                             opencv_imgproc
                             opencv_objdetect)
