PROJECT( FindSilhouette )
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# add_subdirectory(/usr/local/include/GLFW)
# include_directories(/usr/local/include/GLFW)
find_package(OpenGL REQUIRED)



add_subdirectory (external)

include_directories(
	external/glfw-3.2.1/include/
	external/glm-0.9.9.0/
	external/glew-1.13.0/include/
	/usr/local/include/eigen3/
	.
)

set(ALL_LIBS
	${OPENGL_LIBRARY}
	glfw
	GLEW_1130
)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)

find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


ADD_EXECUTABLE( FindSilhouette main.cpp 
	common/shader.cpp
	common/shader.hpp
	common/objloader.hpp
	common/objloader.cpp
	
	SimpleFragmentShader.fragmentshader
	SimpleVertexShader.vertexshader)


TARGET_LINK_LIBRARIES (FindSilhouette ${OpenCV_LIBS} ${ALL_LIBS})

ADD_EXECUTABLE( test optimizer.cpp
	common/objloader.hpp
	common/objloader.cpp)

TARGET_LINK_LIBRARIES (test ${OpenCV_LIBS} ${ALL_LIBS})

