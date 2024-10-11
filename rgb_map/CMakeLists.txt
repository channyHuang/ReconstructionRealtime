FILE(GLOB_RECURSE LIBRARY_FILES_C "*.cpp" "*.hpp")
FILE(GLOB_RECURSE LIBRARY_FILES_H "*.h" "*.inl")

project(rgb_map)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PROJECT_NAME}
    ./../tools
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	)

link_directories(${PROJECT_NAME}
    ${PCL_LIBRARY_DIRS}
    )

add_library(${PROJECT_NAME}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    tools
    optical_flow
)