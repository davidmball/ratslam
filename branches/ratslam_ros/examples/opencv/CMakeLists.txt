
add_definitions(-DRATSLAM_IMPORTS -DRATSLAM_GRAPHICS_IMPORTS)
include_directories("${CMAKE_BINARY_DIR}/include")
link_directories("${CMAKE_BINARY_DIR}/lib")
add_executable(ratslam_opencv_example main.cpp)
target_link_libraries(ratslam_opencv_example
	${RATSLAM_LIBS} ${OpenCV_LIBS})

