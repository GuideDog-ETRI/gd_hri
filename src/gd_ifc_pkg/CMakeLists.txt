cmake_minimum_required(VERSION 3.8)
project(gd_ifc_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in osensor_msgsrder to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GDIMode.msg"
  "msg/GDGGuidancePoint.msg"
  "msg/GDGStatus.msg"
  "msg/GDHDetection2DExt.msg"
  "msg/GDHStatus.msg"
  "msg/GDMDetection.msg"
  "msg/GDMDetections.msg"
  "msg/GDMPrediction.msg"
  "msg/GDMPredictions.msg"
  "msg/GDMSemanticSegmentation.msg"
  "msg/GDMTrack.msg"
  "msg/GDMTracks.msg"
  "msg/GDQHarnessGrip.msg"
  "msg/GDQInitialization.msg"
  "msg/GDQRobotMode.msg"
  "msg/GDQStatus.msg"
  "msg/BboxMsg.msg"
  "msg/DetectedObj.msg"  
  "msg/GdhMsg1.msg"
  "msg/GdhMsg2.msg"
  "msg/GdgMsg1.msg"
  "msg/GdgMsg2.msg"
  "msg/GdmMsg1.msg"
  "msg/GdmMsg2.msg"
  "msg/GdqMsg1.msg"
  "msg/GdqMsg2.msg"
  "msg/UserCommand.msg"
  "srv/GDGGetCourseList.srv"
  "srv/GDGGetPoiList.srv"
  "srv/GDGGuideCourse.srv"
  "srv/GDGResumeGuide.srv"
  "srv/GDHDetectStaticObject.srv"
  "srv/GDHDetectStaticTargetObject.srv"
  "srv/GDHInitializeDetectStaticObject.srv"
  "srv/GDHTerminateDetectStaticObject.srv"
  "srv/GDQEstimatedBaseVelocities.srv"
  "srv/GDGGetImageGuidancePoint.srv"
  "srv/GDGGuideAssist.srv"
  "srv/GDGGuidePoi.srv"
  "srv/GDGStopGuide.srv"
  "srv/GDHDetectStaticObjectAll.srv"
  "srv/GDHExplainPathGP.srv"
  "srv/GDHSpeakCodeID.srv"
  "srv/GDMException.srv"
  "srv/GdhSrv1.srv"
  "srv/GdhSrv2.srv"
  "srv/GdgSrv1.srv"
  "srv/GdgSrv2.srv"
  "srv/GdmSrv1.srv"
  "srv/GdmSrv2.srv"
  "srv/GdqSrv1.srv"
  "srv/GdqSrv2.srv"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs nav_msgs grid_map_msgs vision_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
