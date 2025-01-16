# CMake generated Testfile for 
# Source directory: /home/sharon/catkin_ws/src/robot_pose_ekf
# Build directory: /home/sharon/catkin_ws/build_isolated/robot_pose_ekf
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_robot_pose_ekf_rostest_test_test_robot_pose_ekf.launch "/home/sharon/catkin_ws/build_isolated/robot_pose_ekf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sharon/catkin_ws/build_isolated/robot_pose_ekf/test_results/robot_pose_ekf/rostest-test_test_robot_pose_ekf.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sharon/catkin_ws/src/robot_pose_ekf --package=robot_pose_ekf --results-filename test_test_robot_pose_ekf.xml --results-base-dir \"/home/sharon/catkin_ws/build_isolated/robot_pose_ekf/test_results\" /home/sharon/catkin_ws/src/robot_pose_ekf/test/test_robot_pose_ekf.launch ")
set_tests_properties(_ctest_robot_pose_ekf_rostest_test_test_robot_pose_ekf.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/sharon/catkin_ws/src/robot_pose_ekf/CMakeLists.txt;116;add_rostest;/home/sharon/catkin_ws/src/robot_pose_ekf/CMakeLists.txt;0;")
add_test(_ctest_robot_pose_ekf_rostest_test_test_robot_pose_ekf_zero_covariance.launch "/home/sharon/catkin_ws/build_isolated/robot_pose_ekf/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sharon/catkin_ws/build_isolated/robot_pose_ekf/test_results/robot_pose_ekf/rostest-test_test_robot_pose_ekf_zero_covariance.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sharon/catkin_ws/src/robot_pose_ekf --package=robot_pose_ekf --results-filename test_test_robot_pose_ekf_zero_covariance.xml --results-base-dir \"/home/sharon/catkin_ws/build_isolated/robot_pose_ekf/test_results\" /home/sharon/catkin_ws/src/robot_pose_ekf/test/test_robot_pose_ekf_zero_covariance.launch ")
set_tests_properties(_ctest_robot_pose_ekf_rostest_test_test_robot_pose_ekf_zero_covariance.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/sharon/catkin_ws/src/robot_pose_ekf/CMakeLists.txt;117;add_rostest;/home/sharon/catkin_ws/src/robot_pose_ekf/CMakeLists.txt;0;")
subdirs("gtest")
