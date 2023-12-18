# Install script for directory: /home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ubuntu/workspaces/catkin_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/workspaces/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/workspaces/catkin_ws/install" TYPE PROGRAM FILES "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/workspaces/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/workspaces/catkin_ws/install" TYPE PROGRAM FILES "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/workspaces/catkin_ws/install/setup.bash;/home/ubuntu/workspaces/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/workspaces/catkin_ws/install" TYPE FILE FILES
    "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/setup.bash"
    "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/workspaces/catkin_ws/install/setup.sh;/home/ubuntu/workspaces/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/workspaces/catkin_ws/install" TYPE FILE FILES
    "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/setup.sh"
    "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/workspaces/catkin_ws/install/setup.zsh;/home/ubuntu/workspaces/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/workspaces/catkin_ws/install" TYPE FILE FILES
    "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/setup.zsh"
    "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ubuntu/workspaces/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ubuntu/workspaces/catkin_ws/install" TYPE FILE FILES "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_dashboard_msgs/msg" TYPE FILE FILES
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/msg/ProgramState.msg"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/msg/RobotMode.msg"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/msg/SafetyMode.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_dashboard_msgs/srv" TYPE FILE FILES
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/AddToLog.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/GetLoadedProgram.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/GetProgramState.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/GetRobotMode.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/GetSafetyMode.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/IsInRemoteControl.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/IsProgramRunning.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/IsProgramSaved.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/Load.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/Popup.srv"
    "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/srv/RawRequest.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_dashboard_msgs/action" TYPE FILE FILES "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/action/SetMode.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_dashboard_msgs/msg" TYPE FILE FILES
    "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/ur_dashboard_msgs/msg/SetModeAction.msg"
    "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/ur_dashboard_msgs/msg/SetModeActionGoal.msg"
    "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/ur_dashboard_msgs/msg/SetModeActionResult.msg"
    "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/ur_dashboard_msgs/msg/SetModeActionFeedback.msg"
    "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/ur_dashboard_msgs/msg/SetModeGoal.msg"
    "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/ur_dashboard_msgs/msg/SetModeResult.msg"
    "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/ur_dashboard_msgs/msg/SetModeFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_dashboard_msgs/cmake" TYPE FILE FILES "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/ur_dashboard_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/include/ur_dashboard_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/roseus/ros/ur_dashboard_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/common-lisp/ros/ur_dashboard_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/share/gennodejs/ros/ur_dashboard_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/lib/python3/dist-packages/ur_dashboard_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ubuntu/workspaces/catkin_ws/devel/.private/ur_dashboard_msgs/lib/python3/dist-packages/ur_dashboard_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/ur_dashboard_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_dashboard_msgs/cmake" TYPE FILE FILES "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/ur_dashboard_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_dashboard_msgs/cmake" TYPE FILE FILES
    "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/ur_dashboard_msgsConfig.cmake"
    "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/catkin_generated/installspace/ur_dashboard_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_dashboard_msgs" TYPE FILE FILES "/home/ubuntu/workspaces/catkin_ws/src/Universal_Robots_ROS_Driver/ur_dashboard_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ubuntu/workspaces/catkin_ws/build/ur_dashboard_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
