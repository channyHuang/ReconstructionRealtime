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

set(projectName meshing)
project(${projectName})

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions(-D_USE_BOOST)
add_definitions(-D_USE_EIGEN)
set(CGAL_DIR "E:/thirdLibs/CGAL-5.4-beta1/include" CACHE PATH "")



include_directories(${projectName}
    ${CGAL_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ./../tools
    ./
    )

link_directories(${projectName}
    ${Boost_LIBRARY_DIRS}
    )

add_library(${projectName}
${COMMON_FILES}
${MESH_FILES}
${LIBRARY_FILES}
)

TARGET_LINK_LIBRARIES(${projectName} 
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)