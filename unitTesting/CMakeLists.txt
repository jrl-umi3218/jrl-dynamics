# Olivier Stasse, Francois Keith, JRL, CNRS/AIST
# Creation: 4/11/2008
# History:
#
# Copyright CNRS/AIST

INCLUDE(CTest)

# For tools, we include the same files as the src

# Import jrl-mal flags.
INCLUDE_DIRECTORIES(${JRL_MAL_INCLUDEDIR})

# Import abstract-robot-dynamics flags.
INCLUDE_DIRECTORIES(${ABSTRACT_ROBOT_DYNAMICS_INCLUDEDIR})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/)

ADD_EXECUTABLE(	TestHumanoidDynamicRobot CommonTools.cpp TestHumanoidDynamicRobot.cpp )
SET_TARGET_PROPERTIES(TestHumanoidDynamicRobot
	              PROPERTIES
		      DEPENDS ${CMAKE_BINARY_DIR}/unitTesting/sample.wrl
)

ADD_CUSTOM_COMMAND(TARGET TestHumanoidDynamicRobot POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/unitTesting/sample.wrl
  ${CMAKE_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/unitTesting/sampleSpecificities.xml
  ${CMAKE_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/unitTesting/sampleLinkJointRank.xml
  ${CMAKE_BINARY_DIR}/unitTesting
)

target_link_libraries(TestHumanoidDynamicRobot jrl-dynamics)
ADD_DEPENDENCIES (TestHumanoidDynamicRobot jrl-dynamics)

ADD_EXECUTABLE(	TestHumanoidDynamicRobotDebugLayale CommonTools.cpp TestHumanoidDynamicRobotDebugLayale.cpp)
SET_TARGET_PROPERTIES(TestHumanoidDynamicRobotDebugLayale
	              PROPERTIES
		      DEPENDS ${CMAKE_BINARY_DIR}/unitTesting/sample.wrl
)

ADD_CUSTOM_COMMAND(TARGET TestHumanoidDynamicRobotDebugLayale POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/unitTesting/sample.wrl
  ${CMAKE_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/unitTesting/sampleSpecificities.xml
  ${CMAKE_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/unitTesting/sampleLinkJointRank.xml
  ${CMAKE_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/unitTesting/examples.CMakeList
  ${CMAKE_BINARY_DIR}/unitTesting/CMakeLists.txt
)
target_link_libraries(TestHumanoidDynamicRobotDebugLayale jrl-dynamics)
ADD_DEPENDENCIES (TestHumanoidDynamicRobotDebugLayale jrl-dynamics)

ADD_EXECUTABLE(	TestAbstractInterface
  CommonTools.cpp
  HumanoidCopy.cpp
  TestAbstractInterface.cpp)
target_link_libraries(TestAbstractInterface jrl-dynamics)
ADD_DEPENDENCIES (TestAbstractInterface jrl-dynamics)

ADD_EXECUTABLE(	TestTwoLinksModel
  TwoLinksModel.cpp
  TestTwoLinksModel.cpp )
target_link_libraries(TestTwoLinksModel jrl-dynamics)
ADD_DEPENDENCIES (TestTwoLinksModel jrl-dynamics)

ADD_EXECUTABLE(	Test2Models Test2Models.cpp)
target_link_libraries(Test2Models jrl-dynamics)
ADD_DEPENDENCIES (Test2Models jrl-dynamics)

ADD_EXECUTABLE(	ComputeRealZMP ComputeRealZMP.cpp)
target_link_libraries(ComputeRealZMP jrl-dynamics)
ADD_DEPENDENCIES (ComputeRealZMP jrl-dynamics)

ADD_EXECUTABLE(	ComputeTorqueOnTrajectory ComputeTorqueOnTrajectory.cpp)
target_link_libraries(ComputeTorqueOnTrajectory jrl-dynamics)
ADD_DEPENDENCIES (ComputeTorqueOnTrajectory jrl-dynamics)

ADD_EXECUTABLE(	TestFreeFlyer TestFreeFlyer.cpp CommonTools.cpp)
target_link_libraries(TestFreeFlyer jrl-dynamics)
ADD_DEPENDENCIES(TestFreeFlyer jrl-dynamics)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/unitTesting/output.ascii.cmake
		 ${CMAKE_BINARY_DIR}/unitTesting/output.ascii )

ADD_TEST(TestHumanoidDynamicRobot TestHumanoidDynamicRobot "output.ascii")

ADD_TEST(TestAbstractInterface TestAbstractInterface)

ADD_TEST(TestTwoLinksModel TestTwoLinksModel)

SET(${PROJECT_NAME}_EXAMPLES
  CommonTools.cpp
  CommonTools.h
  TestHumanoidDynamicRobot.cpp
  TestAbstractInterface.cpp
  TestFreeFlyer.cpp
  ComputeRealZMP.cpp
  TwoLinksModel.cpp
  TwoLinksModel.h
  TestTwoLinksModel.cpp
  HumanoidCopy.cpp
  HumanoidCopy.h
  )

INSTALL(FILES ${${PROJECT_NAME}_EXAMPLES}
  DESTINATION share/jrl/dynamics/examples/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

INSTALL(FILES ${CMAKE_BINARY_DIR}/unitTesting/CMakeLists.txt
  DESTINATION share/jrl/dynamics/examples/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

SET(${PROJECT_NAME}_EXAMPLES_DATA
  output.ascii.cmake
  sample.wrl
  sampleSpecificities.xml
  sampleLinkJointRank.xml
  sampleInitConfig.dat
  )
INSTALL(FILES ${${PROJECT_NAME}_EXAMPLES_DATA}
  DESTINATION share/jrl/dynamics/examples/data
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

