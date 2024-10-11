CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)

project(osgReconRealtime)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	set(CGAL_DIR "/usr/include/CGAL" CACHE PATH "")
	set(AirSim_DIR "/home/channy/Documents/thirdlibs/AirSim" CACHE PATH "/home/channy/Documents/thirdlibs/AirSim")
	find_package(ffmpeg)
else()
	set(ffmpeg_DIR "E:/thirdLibs/installers/ffmpeg-6.0" CACHE PATH "")
	set(AirSim_DIR "E:/thirdLibs/AirSim" CACHE PATH "E:/thirdLibs/AirSim")
	set(CGAL_DIR "E:/thirdLibs/CGAL-5.4-beta1/include" CACHE PATH "")
endif()

# 依赖库
find_package(OpenSceneGraph REQUIRED osg osgViewer osgUtil osgDB osgGA osgText)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
find_package(OpenMP REQUIRED)
endif()

# 自定义通用类
set(OUTER_DIR ${CMAKE_SOURCE_DIR}/../commonClass)

add_subdirectory(tools)
add_subdirectory(loam)
add_subdirectory(optical_flow)
add_subdirectory(rgb_map)
add_subdirectory(meshing)
add_subdirectory(${OUTER_DIR}/imgui imgui)
add_subdirectory(${OUTER_DIR}/commonOsg commonOsg)

add_definitions(-D_USE_BOOST)
add_definitions(-D_USE_EIGEN)

# message(WARNING "debug info: " ${Boost_LIBRARY_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${PROJECT_NAME}
    ${OUTER_DIR}

	${OSG_INCLUDE_DIR}
	${GLFW3_INCLUDE_DIR}
	#${ffmpeg_DIR}/include
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
    ${CGAL_DIR}
    ${Boost_INCLUDE_DIRS}
    ./tools
    ./loam/include
    ./rgb_map
	./
    #airsim
    ${AirSim_DIR}/AirLib/deps/rpclib/include
    ${AirSim_DIR}/AirLib/include
	)

file (GLOB ROOT_FILES *.h *.cpp)
#source_group(tools FILES ${TOOLS_FILES})

set(CMAKE_BUILD_TYPE "Release")
link_directories(${PROJECT_NAME}
	${GLFW3_LIBRARY_DIR}
	${OPENSCENEGRAPH_LIBRARIES}
	${ffmpeg_DIR}/lib
	${PCL_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
    #AirSim
    # ${AirSim_DIR}/AirLib/deps/lib/x64/Release
    # ${AirSim_DIR}/AirLib/deps/rpclib/lib/x64/Release
    # ${AirSim_DIR}/MavLinkCom/lib/x64/Release
    # ${AirSim_DIR}/AirLib/lib/x64/Release
	)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    link_directories(${PROJECT_NAME}
        ${AIRSIM_DIR}/build_release/output/lib
    )
else()
    link_directories(${PROJECT_NAME}
        ${AIRSIM_DIR}/AirLib/deps/rpclib/lib
        ${AIRSIM_DIR}/MavLinkCom/lib/x64/Release
        ${AIRSIM_DIR}/AirLib/lib/x64/Release
    )
endif()

add_executable(${PROJECT_NAME}
${ROOT_FILES}
)

set (FFMPEGLIBS 
	avcodec
	avdevice
	avfilter
	avformat
	avutil
	postproc
	swresample
	swscale
)

target_link_libraries(${PROJECT_NAME}
		imgui
		commonOsg
		
		${OPENSCENEGRAPH_LIBRARIES}
		${OpenCV_LIBS}
		${PCL_LIBRARIES}

		tools
		loam
		optical_flow
		rgb_map
		meshing
# AirSim
AirLib
rpc
MavLinkCom
)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
target_link_libraries(${PROJECT_NAME}
glfw
OpenMP::OpenMP_CXX
Boost::filesystem
Boost::iostreams
Boost::program_options
Boost::system
Boost::serialization
gmp
)
elseif()
target_link_libraries(${PROJECT_NAME}
	opengl32
	glfw3

	${FFMPEGLIBS}
)
endif()