FILE(GLOB_RECURSE LIBRARY_FILES_C "*.cpp" "*.hpp")
FILE(GLOB_RECURSE LIBRARY_FILES_H "*.h" "*.inl")

set(projectName loam)
project(${projectName})

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${projectName}
    ${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ./include
    ./../tools
	)

link_directories(${projectName}
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBS}
    )

add_library(${projectName}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${projectName} 
    ${PCL_LIBRARIES}
    tools
)