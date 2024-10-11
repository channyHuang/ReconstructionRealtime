FILE(GLOB_RECURSE LIBRARY_FILES_C "*.cpp" "*.hpp")
FILE(GLOB_RECURSE LIBRARY_FILES_H "*.h" "*.inl")

project(tools)

find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_NAME}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PROJECT_NAME}

    )

add_library(${PROJECT_NAME}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 

)