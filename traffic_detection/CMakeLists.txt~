
cmake_minimum_required(VERSION 2.8)

project( TrafficDetection )
find_package( OpenCV REQUIRED )
set( OPENNI2_INCLUDE /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Include)
set( OPENNI2_REDIST  /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Redist)
include_directories( ${OPENNI2_INCLUDE} )
link_directories ( ${OPENNI2_REDIST} )

set( source_files
    DetecterTrafficSign_NII.h
    DetecterTrafficSign_NII.cpp
)

set( lib_name traffic-detection)
add_library( ${lib_name} ${source_files} )
target_link_libraries( ${lib_name} ${OpenCV_LIBS})


set( test_exec_name test-traffic-detection)
add_executable( ${test_exec_name}  test-traffic-detection.cpp )

target_link_libraries( ${test_exec_name} ${lib_name} ${OpenCV_LIBS})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(traffic-detection ${PYTHON_LIBRARIES})
