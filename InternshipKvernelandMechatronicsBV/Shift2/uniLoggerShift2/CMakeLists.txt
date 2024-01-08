SET(execName uniLogger)
CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
#PROJECT(uniLogger LANGUAGES CXX CUDA)

# Assign cuda compiler
if (NOT CMAKE_CUDA_COMPILER)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-10.2/bin/nvcc")
endif()

PROJECT(${execName} LANGUAGES CXX CUDA)


option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

IF(NOT WIN32)
    SET(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-Wno-write-strings -fpermissive)
ENDIF()

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)  
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
SET(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)

#cuPCL directory 
set(CUPCL_DIR "/usr/local/cuPCL")


if(PCL_FOUND)

    #redefine the matplotplusplus library directory as an relative sub-directory ./matplotplusplus
    add_subdirectory(/usr/local/matplotplusplus matplotplusplus)
 
    include_directories(${ZED_INCLUDE_DIRS})
    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(${GLEW_INCLUDE_DIRS})
    include_directories(${GLUT_INCLUDE_DIRS})
    include_directories(${CUDA_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    include_directories(${CUPCL_DIR}/include)
    include_directories(include)

    link_directories(${ZED_LIBRARY_DIR})
    link_directories(${OpenCV_LIBRARY_DIRS})
    link_directories(${GLEW_LIBRARY_DIRS})
    link_directories(${GLUT_LIBRARY_DIRS})
    link_directories(${OpenGL_LIBRARY_DIRS})
    link_directories(${CUDA_LIBRARY_DIRS})
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
    link_directories(${PCL_LIBRARY_DIRS})

    find_library(CUPCL_CLUSTER_LIB NAMES cudacluster PATHS "${CUPCL_DIR}/lib" PATH_SUFFIXES lib NO_DEFAULT_PATH)
    find_library(CUPCL_FILTER_LIB NAMES cudafilter PATHS "${CUPCL_DIR}/lib" PATH_SUFFIXES lib NO_DEFAULT_PATH)
    find_library(CUPCL_ICP_LIB NAMES cudaicp PATHS "${CUPCL_DIR}/lib" PATH_SUFFIXES lib NO_DEFAULT_PATH)
    find_library(CUPCL_NDT_LIB NAMES cudandt PATHS "${CUPCL_DIR}/lib" PATH_SUFFIXES lib NO_DEFAULT_PATH)
    find_library(CUPCL_OCTREE_LIB NAMES cudaoctree PATHS "${CUPCL_DIR}/lib" PATH_SUFFIXES lib NO_DEFAULT_PATH)
    find_library(CUPCL_SEGMENTATION_LIB NAMES cudasegmentation PATHS "${CUPCL_DIR}/lib" PATH_SUFFIXES lib NO_DEFAULT_PATH)
    set(CUPCL_LIBS ${CUPCL_CLUSTER_LIB} ${CUPCL_FILTER_LIB} ${CUPCL_ICP_LIB} ${CUPCL_NDT_LIB} ${CUPCL_OCTREE_LIB} ${CUPCL_SEGMENTATION_LIB})

   
    #FILE(GLOB_RECURSE SRC_FILES src/seedbed_quality_control.cpp src/CAN_communication.cpp src/zedCamera.cpp src/zed_data_grabber.cpp src/thread_utilities.cpp src/data_processing.cpp src/utilities.cpp src/plane_transform.cpp src/surface_quality_detection.cpp src/IIR_filter.cpp src/radius_outlier_removal.cpp src/linear_regression.cpp src/visualisation.cpp) 
    FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
    #FILE(GLOB_RECURSE HDR_FILES include/*.hpp)
    #ADD_EXECUTABLE(${PROJECT_NAME} ${SRC_FILES}) #${HDR_FILES}
    ADD_EXECUTABLE(${execName} ${SRC_FILES})

    add_definitions(-std=c++14 -g -O3)
    list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") ## needed if you install pcl from repo (and libproj-dev)

    if (LINK_SHARED_ZED)
        SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${OpenCV_LIBRARIES})
    else()
        SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
    endif()

    TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                            ${ZED_LIBS} 
                            ${OpenCV_LIBRARIES}
                            ${OPENGL_LIBRARIES}
                            ${GLUT_LIBRARIES}
                            ${GLEW_LIBRARIES}
                            ${PCL_LIBRARIES}
                            ${SPECIAL_OS_LIBS}
                            ${CUPCL_LIBS}
                            matplot)            
endif()
