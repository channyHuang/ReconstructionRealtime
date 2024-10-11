FILE(GLOB_RECURSE LIBRARY_FILES_C "*.cpp" "*.hpp")
FILE(GLOB_RECURSE LIBRARY_FILES_H "*.h" "*.inl")

project(loam)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PROJECT_NAME}
    ${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ./include
    ./../tools
	)

link_directories(${PROJECT_NAME}
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBS}
    )

add_library(${PROJECT_NAME}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
    ${PCL_LIBRARIES}
    tools
)