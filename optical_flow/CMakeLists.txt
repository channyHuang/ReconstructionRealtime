FILE(GLOB_RECURSE LIBRARY_FILES_C "*.cpp" "*.hpp")
FILE(GLOB_RECURSE LIBRARY_FILES_H "*.h" "*.inl")

project(optical_flow)

find_package(OpenCV REQUIRED)

include_directories(${PROJECT_NAME}
	${OpenCV_INCLUDE_DIRS}
    ./../tools
	)

link_directories(${PROJECT_NAME}
    
    )

add_library(${PROJECT_NAME}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
    ${OpenCV_LIBS}
    tools
)