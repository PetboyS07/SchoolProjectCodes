<<<<<<<< HEAD:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerCodeWorking/build/cmake_install.cmake
# Install script for directory: /home/user/repos/UniLogWorkspace/uniLoggerShift2
========
# Install script for directory: /home/user/repos/UniLogWorkspace/workingprogram
>>>>>>>> 71cccea26b3de34f671a0b0d3ea02f86ae34e0e1:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerShift2/build/cmake_install.cmake

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
<<<<<<<< HEAD:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerCodeWorking/build/cmake_install.cmake
  include("/home/user/repos/UniLogWorkspace/uniLoggerShift2/build/matplotplusplus/cmake_install.cmake")
========
  include("/home/user/repos/UniLogWorkspace/workingprogram/build/matplotplusplus/cmake_install.cmake")
>>>>>>>> 71cccea26b3de34f671a0b0d3ea02f86ae34e0e1:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerShift2/build/cmake_install.cmake
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<<< HEAD:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerCodeWorking/build/cmake_install.cmake
file(WRITE "/home/user/repos/UniLogWorkspace/uniLoggerShift2/build/${CMAKE_INSTALL_MANIFEST}"
========
file(WRITE "/home/user/repos/UniLogWorkspace/workingprogram/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>>> 71cccea26b3de34f671a0b0d3ea02f86ae34e0e1:InternshipKvernelandMechatronicsBV/Shift2/uniLoggerShift2/build/cmake_install.cmake
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
