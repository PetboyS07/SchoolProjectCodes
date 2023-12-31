# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/repos/intern_patrick_schuurman/uniLoggerMain

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/repos/intern_patrick_schuurman/uniLoggerMain/build

# Include any dependencies generated for this target.
include CMakeFiles/uniLog.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/uniLog.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/uniLog.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/uniLog.dir/flags.make

CMakeFiles/uniLog.dir/src/GLViewer.cpp.o: CMakeFiles/uniLog.dir/flags.make
CMakeFiles/uniLog.dir/src/GLViewer.cpp.o: /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/GLViewer.cpp
CMakeFiles/uniLog.dir/src/GLViewer.cpp.o: CMakeFiles/uniLog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/repos/intern_patrick_schuurman/uniLoggerMain/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/uniLog.dir/src/GLViewer.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/uniLog.dir/src/GLViewer.cpp.o -MF CMakeFiles/uniLog.dir/src/GLViewer.cpp.o.d -o CMakeFiles/uniLog.dir/src/GLViewer.cpp.o -c /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/GLViewer.cpp

CMakeFiles/uniLog.dir/src/GLViewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uniLog.dir/src/GLViewer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/GLViewer.cpp > CMakeFiles/uniLog.dir/src/GLViewer.cpp.i

CMakeFiles/uniLog.dir/src/GLViewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uniLog.dir/src/GLViewer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/GLViewer.cpp -o CMakeFiles/uniLog.dir/src/GLViewer.cpp.s

CMakeFiles/uniLog.dir/src/cameraControl.cpp.o: CMakeFiles/uniLog.dir/flags.make
CMakeFiles/uniLog.dir/src/cameraControl.cpp.o: /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/cameraControl.cpp
CMakeFiles/uniLog.dir/src/cameraControl.cpp.o: CMakeFiles/uniLog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/repos/intern_patrick_schuurman/uniLoggerMain/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/uniLog.dir/src/cameraControl.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/uniLog.dir/src/cameraControl.cpp.o -MF CMakeFiles/uniLog.dir/src/cameraControl.cpp.o.d -o CMakeFiles/uniLog.dir/src/cameraControl.cpp.o -c /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/cameraControl.cpp

CMakeFiles/uniLog.dir/src/cameraControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uniLog.dir/src/cameraControl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/cameraControl.cpp > CMakeFiles/uniLog.dir/src/cameraControl.cpp.i

CMakeFiles/uniLog.dir/src/cameraControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uniLog.dir/src/cameraControl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/cameraControl.cpp -o CMakeFiles/uniLog.dir/src/cameraControl.cpp.s

CMakeFiles/uniLog.dir/src/main.cpp.o: CMakeFiles/uniLog.dir/flags.make
CMakeFiles/uniLog.dir/src/main.cpp.o: /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/main.cpp
CMakeFiles/uniLog.dir/src/main.cpp.o: CMakeFiles/uniLog.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/repos/intern_patrick_schuurman/uniLoggerMain/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/uniLog.dir/src/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/uniLog.dir/src/main.cpp.o -MF CMakeFiles/uniLog.dir/src/main.cpp.o.d -o CMakeFiles/uniLog.dir/src/main.cpp.o -c /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/main.cpp

CMakeFiles/uniLog.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uniLog.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/main.cpp > CMakeFiles/uniLog.dir/src/main.cpp.i

CMakeFiles/uniLog.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uniLog.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/repos/intern_patrick_schuurman/uniLoggerMain/src/main.cpp -o CMakeFiles/uniLog.dir/src/main.cpp.s

# Object files for target uniLog
uniLog_OBJECTS = \
"CMakeFiles/uniLog.dir/src/GLViewer.cpp.o" \
"CMakeFiles/uniLog.dir/src/cameraControl.cpp.o" \
"CMakeFiles/uniLog.dir/src/main.cpp.o"

# External object files for target uniLog
uniLog_EXTERNAL_OBJECTS =

uniLog: CMakeFiles/uniLog.dir/src/GLViewer.cpp.o
uniLog: CMakeFiles/uniLog.dir/src/cameraControl.cpp.o
uniLog: CMakeFiles/uniLog.dir/src/main.cpp.o
uniLog: CMakeFiles/uniLog.dir/build.make
uniLog: /usr/local/zed/lib/libsl_zed.so
uniLog: /usr/lib/aarch64-linux-gnu/libopenblas.so
uniLog: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
uniLog: /usr/lib/aarch64-linux-gnu/libcuda.so
uniLog: /usr/local/cuda/lib64/libcudart.so
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_gapi.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libOpenGL.so
uniLog: /usr/lib/aarch64-linux-gnu/libGLX.so
uniLog: /usr/lib/aarch64-linux-gnu/libGLU.so
uniLog: /usr/lib/aarch64-linux-gnu/libglut.so
uniLog: /usr/lib/aarch64-linux-gnu/libXmu.so
uniLog: /usr/lib/aarch64-linux-gnu/libXi.so
uniLog: /usr/lib/aarch64-linux-gnu/libGLEW.so
uniLog: /usr/local/lib/libpcl_keypoints.so
uniLog: /usr/local/lib/libpcl_tracking.so
uniLog: /usr/local/lib/libpcl_recognition.so
uniLog: /usr/local/lib/libpcl_stereo.so
uniLog: /usr/local/lib/libpcl_cuda_features.so
uniLog: /usr/local/lib/libpcl_cuda_segmentation.so
uniLog: /usr/local/lib/libpcl_cuda_sample_consensus.so
uniLog: /usr/local/lib/libpcl_outofcore.so
uniLog: /usr/local/lib/libpcl_gpu_features.so
uniLog: /usr/local/lib/libpcl_gpu_kinfu.so
uniLog: /usr/local/lib/libpcl_gpu_kinfu_large_scale.so
uniLog: /usr/local/lib/libpcl_gpu_segmentation.so
uniLog: /usr/local/lib/libpcl_people.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_system.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_iostreams.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_serialization.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_regex.so
uniLog: /usr/lib/libOpenNI.so
uniLog: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
uniLog: /usr/lib/libOpenNI2.so
uniLog: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
uniLog: /usr/lib/aarch64-linux-gnu/libfreetype.so
uniLog: /usr/lib/aarch64-linux-gnu/libz.so
uniLog: /usr/lib/aarch64-linux-gnu/libjpeg.so
uniLog: /usr/lib/aarch64-linux-gnu/libpng.so
uniLog: /usr/lib/aarch64-linux-gnu/libtiff.so
uniLog: /usr/lib/aarch64-linux-gnu/libflann_cpp.so
uniLog: /usr/lib/aarch64-linux-gnu/libqhull_r.so
uniLog: /usr/local/zed/lib/libsl_zed.so
uniLog: /usr/lib/aarch64-linux-gnu/libopenblas.so
uniLog: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
uniLog: /usr/lib/aarch64-linux-gnu/libcuda.so
uniLog: /usr/local/cuda/lib64/libcudart.so
uniLog: /usr/lib/aarch64-linux-gnu/libOpenGL.so
uniLog: /usr/lib/aarch64-linux-gnu/libGLX.so
uniLog: /usr/lib/aarch64-linux-gnu/libglut.so
uniLog: /usr/lib/aarch64-linux-gnu/libXmu.so
uniLog: /usr/lib/aarch64-linux-gnu/libXi.so
uniLog: /usr/lib/aarch64-linux-gnu/libGLEW.so
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.1.1
uniLog: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.1.1
uniLog: /usr/local/lib/libpcl_registration.so
uniLog: /usr/local/lib/libpcl_surface.so
uniLog: /usr/local/lib/libpcl_gpu_octree.so
uniLog: /usr/local/lib/libpcl_gpu_utils.so
uniLog: /usr/local/lib/libpcl_gpu_containers.so
uniLog: /usr/local/lib/libpcl_segmentation.so
uniLog: /usr/local/lib/libpcl_features.so
uniLog: /usr/local/lib/libpcl_filters.so
uniLog: /usr/local/lib/libpcl_sample_consensus.so
uniLog: /usr/local/lib/libpcl_ml.so
uniLog: /usr/local/lib/libpcl_visualization.so
uniLog: /usr/local/lib/libpcl_search.so
uniLog: /usr/local/lib/libpcl_kdtree.so
uniLog: /usr/local/lib/libpcl_io.so
uniLog: /usr/local/lib/libpcl_octree.so
uniLog: /usr/lib/libOpenNI.so
uniLog: /usr/lib/aarch64-linux-gnu/libusb-1.0.so
uniLog: /usr/lib/libOpenNI2.so
uniLog: /usr/lib/aarch64-linux-gnu/libvtkChartsCore-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkInfovisCore-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkInteractionImage-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libjpeg.so
uniLog: /usr/lib/aarch64-linux-gnu/libpng.so
uniLog: /usr/lib/aarch64-linux-gnu/libtiff.so
uniLog: /usr/lib/aarch64-linux-gnu/libvtkIOLegacy-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkIOPLY-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkIOGeometry-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkRenderingLOD-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkViewsContext2D-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkRenderingContext2D-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkViewsCore-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkInteractionWidgets-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkFiltersModeling-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkFiltersHybrid-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkImagingGeneral-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkImagingSources-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkRenderingAnnotation-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkftgl-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libfreetype.so
uniLog: /usr/lib/aarch64-linux-gnu/libvtkImagingColor-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkRenderingVolume-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkGUISupportQt-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libGLU.so
uniLog: /usr/lib/aarch64-linux-gnu/libvtkRenderingCore-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonColor-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkFiltersExtraction-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkFiltersStatistics-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkImagingFourier-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkalglib-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkFiltersGeometry-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkFiltersSources-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkFiltersGeneral-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkFiltersCore-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonComputationalGeometry-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkImagingHybrid-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkImagingCore-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkIOImage-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkDICOMParser-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkIOCore-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonExecutionModel-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonMisc-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonSystem-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtksys-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonTransforms-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonMath-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkCommonCore-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libvtkmetaio-6.3.so.6.3.0
uniLog: /usr/lib/aarch64-linux-gnu/libz.so
uniLog: /usr/lib/aarch64-linux-gnu/libGL.so
uniLog: /usr/lib/aarch64-linux-gnu/libSM.so
uniLog: /usr/lib/aarch64-linux-gnu/libICE.so
uniLog: /usr/lib/aarch64-linux-gnu/libX11.so
uniLog: /usr/lib/aarch64-linux-gnu/libXext.so
uniLog: /usr/lib/aarch64-linux-gnu/libXt.so
uniLog: /usr/lib/aarch64-linux-gnu/libQt5Widgets.so.5.9.5
uniLog: /usr/lib/aarch64-linux-gnu/libQt5Gui.so.5.9.5
uniLog: /usr/lib/aarch64-linux-gnu/libQt5Core.so.5.9.5
uniLog: /usr/local/lib/libpcl_common.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_system.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_iostreams.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_serialization.so
uniLog: /usr/lib/aarch64-linux-gnu/libboost_regex.so
uniLog: CMakeFiles/uniLog.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/repos/intern_patrick_schuurman/uniLoggerMain/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable uniLog"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/uniLog.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/uniLog.dir/build: uniLog
.PHONY : CMakeFiles/uniLog.dir/build

CMakeFiles/uniLog.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/uniLog.dir/cmake_clean.cmake
.PHONY : CMakeFiles/uniLog.dir/clean

CMakeFiles/uniLog.dir/depend:
	cd /home/user/repos/intern_patrick_schuurman/uniLoggerMain/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/repos/intern_patrick_schuurman/uniLoggerMain /home/user/repos/intern_patrick_schuurman/uniLoggerMain /home/user/repos/intern_patrick_schuurman/uniLoggerMain/build /home/user/repos/intern_patrick_schuurman/uniLoggerMain/build /home/user/repos/intern_patrick_schuurman/uniLoggerMain/build/CMakeFiles/uniLog.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/uniLog.dir/depend

