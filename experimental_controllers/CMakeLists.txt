cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(experimental_controllers
  src/cartesian_pose_twist_controller.cpp
  src/cartesian_spline_trajectory_controller.cpp
  src/trajectory_controller.cpp
  src/pid_position_velocity_controller.cpp
  src/cartesian_trajectory_controller.cpp
  src/joint_pd_controller.cpp
  src/cartesian_wrench_controller.cpp
  src/cartesian_tff_controller.cpp
  src/cartesian_twist_controller_ik.cpp
#  src/cartesian_hybrid_controller.cpp
#  src/joint_position_smoothing_controller.cpp
  src/joint_inverse_dynamics_controller.cpp
#  src/joint_autotuner.cpp
#  src/joint_calibration_controller.cpp
#  src/joint_blind_calibration_controller.cpp
#  src/joint_chain_constraint_controller.cpp
#  src/joint_chain_sine_controller.cpp
#  src/probe.cpp

  src/cartesian_twist_controller_ik_with_nullspace_optimization.cpp
  src/joint_velocity_filtered_controller.cpp

  src/joint_velocity_filtered_controller_tuner.cpp

#  src/head_servoing_controller.cpp
#  src/arm_trajectory_controller.cpp
  src/joint_trajectory_controller.cpp
#  src/pr2_gripper_controller.cpp

  src/base_position_pid.cpp

  src/texas_odometry.cpp
)
target_link_libraries(experimental_controllers ltdl)

rosbuild_add_executable(test_trajectory_controller test/test_trajectory_srv.cpp)
target_link_libraries(test_trajectory_controller experimental_controllers)

rosbuild_add_executable(base_trajectory_controller src/base_trajectory_controller.cpp)
