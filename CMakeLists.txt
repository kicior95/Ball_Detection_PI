cmake_minimum_required(VERSION 2.8)

project(Ball_Detecting)
add_executable(${PROJECT_NAME} "main.cpp")

find_package(OpenCV)
find_package(raspicam REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} libwiringPi.so ${raspicam_CV_LIBS})
