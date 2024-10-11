CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)

project(meshing)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-D_USE_BOOST)
add_definitions(-D_USE_EIGEN)

# includes
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(CGAL REQUIRED)
	set(CGAL_DIR "/usr/include/" CACHE PATH "")
else()
	set(CGAL_DIR "E:/thirdLibs/CGAL-5.4-beta1/include" CACHE PATH "")
endif()

# files
FILE(GLOB_RECURSE LIBRARY_FILES_C "*.hpp")
FILE(GLOB LIBRARY_FILES_H "*.h" )

FILE(GLOB COMMON_FILES 
    ./MVS/Common/Common.cpp
    ./MVS/Common/Timer.cpp
    ./MVS/Common/Types.cpp
    )

FILE(GLOB MESH_FILES 
    ./MVS/Camera.cpp
    ./MVS/IBFS.cpp
    ./MVS/Image.cpp
    ./MVS/Mesh.cpp
    ./MVS/OBJ.cpp
    ./MVS/Platform.cpp
    ./MVS/PointCloud.cpp
    )
    
FILE(GLOB LIBRARY_FILES
    )
    
source_group(Common FILES ${COMMON_FILES})

# path
include_directories(${PROJECT_NAME}
    ${CGAL_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ./../tools
    ./
    )

link_directories(${PROJECT_NAME}
    ${Boost_LIBRARY_DIRS}
    )

add_library(${PROJECT_NAME}
    ${COMMON_FILES}
    ${MESH_FILES}
    ${LIBRARY_FILES}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
${OpenCV_LIBS}    
${Boost_LIBRARIES}
)
