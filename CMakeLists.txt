cmake_minimum_required(VERSION 3.12)

project(Ball_Detecting)
add_executable(${PROJECT_NAME} "main.cpp")
find_package(PythonLibs 2.7)
find_package(OpenCV)
find_package(raspicam REQUIRED)

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} libwiringPi.so ${raspicam_CV_LIBS} ${PYTHON_LIBRARIES})
