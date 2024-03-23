FILE(GLOB_RECURSE LIBRARY_FILES_C "*.cpp" "*.hpp")
FILE(GLOB_RECURSE LIBRARY_FILES_H "*.h" "*.inl")

set(projectName rgb_map)
project(${projectName})

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${projectName}
    ./../tools
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	)

link_directories(${projectName}
    ${PCL_LIBRARY_DIRS}
    )

add_library(${projectName}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${projectName} 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    tools
    optical_flow
)