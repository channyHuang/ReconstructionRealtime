FILE(GLOB_RECURSE LIBRARY_FILES_C "*.cpp" "*.hpp")
FILE(GLOB_RECURSE LIBRARY_FILES_H "*.h" "*.inl")

set(projectName tools)
project(${projectName})

find_package(Eigen3 REQUIRED)

include_directories(${projectName}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(${projectName}

    )

add_library(${projectName}
${LIBRARY_FILES_C} 
${LIBRARY_FILES_H}
)

TARGET_LINK_LIBRARIES(${projectName} 

)