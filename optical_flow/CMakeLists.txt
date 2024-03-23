FILE(GLOB_RECURSE LIBRARY_FILES_C "*.cpp" "*.hpp")
FILE(GLOB_RECURSE LIBRARY_FILES_H "*.h" "*.inl")

set(projectName optical_flow)
project(${projectName})

find_package(OpenCV REQUIRED)

include_directories(${projectName}
	${OpenCV_INCLUDE_DIRS}
    ./../tools
	)

link_directories(${projectName}
    
    )

add_library(${projectName}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${projectName} 
    ${OpenCV_LIBS}
    tools
)