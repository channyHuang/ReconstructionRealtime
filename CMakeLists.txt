CMAKE_MINIMUM_REQUIRED(VERSION 3.8.2)

set(projectName osgReconRealtime)
project(${projectName})

set(OUTER_DIR ${CMAKE_SOURCE_DIR}/../commonClass)
set(ffmpeg_DIR "E:/thirdLibs/installers/ffmpeg-6.0" CACHE PATH "")
set(CGAL_DIR "E:/thirdLibs/CGAL-5.4-beta1/include" CACHE PATH "")
set(AirSim_DIR "E:/thirdLibs/AirSim")

add_subdirectory(tools)
add_subdirectory(loam)
add_subdirectory(optical_flow)
add_subdirectory(rgb_map)
add_subdirectory(meshing)
add_subdirectory(${OUTER_DIR}/imgui imgui)
add_subdirectory(${OUTER_DIR}/commonOsg commonOsg)

add_definitions(-D_USE_BOOST)
add_definitions(-D_USE_EIGEN)

find_package(OpenSceneGraph REQUIRED osg osgViewer osgUtil osgDB osgGA osgText)
find_package(GLFW3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)

message(WARNING "debug info: " ${Boost_LIBRARY_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${projectName}
    ${OUTER_DIR}

	${OSG_INCLUDE_DIR}
	${GLFW3_INCLUDE_DIR}
	${ffmpeg_DIR}/include
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
    ${CGAL_DIR}
    ${Boost_INCLUDE_DIRS}
    ./tools
    ./loam/include
    ./rgb_map
    #airsim
    ${AirSim_DIR}/AirLib/deps/rpclib/include
    ${AirSim_DIR}/AirLib/include
	)

file (GLOB ROOT_FILES *.h *.cpp)
#source_group(tools FILES ${TOOLS_FILES})

set(CMAKE_BUILD_TYPE "Release")
link_directories(${projectName}
	${GLFW3_LIBRARY_DIR}
	${OPENSCENEGRAPH_LIBRARIES}
	${ffmpeg_DIR}/lib
	${PCL_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
    #AirSim
    ${AirSim_DIR}/AirLib/deps/lib/x64/Release
    ${AirSim_DIR}/AirLib/deps/rpclib/lib/x64/Release
    ${AirSim_DIR}/MavLinkCom/lib/x64/Release
    ${AirSim_DIR}/AirLib/lib/x64/Release
	)

add_executable(${projectName}
${ROOT_FILES}
)

target_link_libraries(${projectName}
	imgui
	commonOsg
	opengl32
	glfw3
	${OPENSCENEGRAPH_LIBRARIES}
	${OpenCV_LIBS}
	avcodec
	avdevice
	avfilter
	avformat
	avutil
	postproc
	swresample
	swscale
	${PCL_LIBRARIES}
    tools
    loam
    optical_flow
    rgb_map
    meshing
    # AirSim
    rpc
    MavLinkCom
    AirLib
)
