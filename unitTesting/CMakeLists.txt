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
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/private)

# JRL_DYNAMICS_ADD_TEST(TEST)
# ---------------------------
#
# - Declare a new program called ``TEST''.
# - Compile it and link it against jrl-dynamics.
# - Add it to the TEST suite.
#
# TEST : test name
#
MACRO(JRL_DYNAMICS_ADD_TEST TEST)
	ADD_EXECUTABLE(${TEST} ${TEST}.cpp)
	SET_TARGET_PROPERTIES(${TEST}
		PROPERTIES
		COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
	)
	ADD_TEST(${TEST} ${TEST})
	#MESSAGE(FATAL_ERROR ${${PROJECT_NAME}_test_CXXFLAGS})
ENDMACRO(JRL_DYNAMICS_ADD_TEST TEST)

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
PKG_CONFIG_USE_DEPENDENCY(TestHumanoidDynamicRobot jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(TestHumanoidDynamicRobot abstract-robot-dynamics)

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
PKG_CONFIG_USE_DEPENDENCY(TestHumanoidDynamicRobotDebugLayale jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(TestHumanoidDynamicRobotDebugLayale abstract-robot-dynamics)

ADD_DEPENDENCIES (TestHumanoidDynamicRobotDebugLayale ${PROJECT_NAME})


#ADD_EXECUTABLE(	TestSot CommonTools.cpp TestSot.cpp)
#ADD_DEPENDENCIES (TestSot ${PROJECT_NAME})
#SET_TARGET_PROPERTIES(TestSot
#  PROPERTIES
#  COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
#  LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
#  DEPENDS ${${PROJECT_NAME}_BINARY_DIR}/unitTesting/sample.wrl
#  )


ADD_EXECUTABLE(	TestAbstractInterface
  CommonTools.cpp
  HumanoidCopy.cpp
  TestAbstractInterface.cpp)
target_link_libraries(TestAbstractInterface jrl-dynamics)
ADD_DEPENDENCIES (TestAbstractInterface jrl-dynamics)
PKG_CONFIG_USE_DEPENDENCY(TestAbstractInterface jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(TestAbstractInterface abstract-robot-dynamics)


ADD_EXECUTABLE(	TestTwoLinksModel
  TwoLinksModel.cpp
  TestTwoLinksModel.cpp )
target_link_libraries(TestTwoLinksModel jrl-dynamics)
ADD_DEPENDENCIES (TestTwoLinksModel jrl-dynamics)
PKG_CONFIG_USE_DEPENDENCY(TestTwoLinksModel jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(TestTwoLinksModel abstract-robot-dynamics)

ADD_EXECUTABLE(	Test2Models Test2Models.cpp)
target_link_libraries(Test2Models jrl-dynamics)
ADD_DEPENDENCIES (Test2Models jrl-dynamics)
PKG_CONFIG_USE_DEPENDENCY(Test2Models jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(Test2Models abstract-robot-dynamics)

ADD_EXECUTABLE(	ComputeRealZMP ComputeRealZMP.cpp)
target_link_libraries(ComputeRealZMP jrl-dynamics)
ADD_DEPENDENCIES (ComputeRealZMP jrl-dynamics)
PKG_CONFIG_USE_DEPENDENCY(ComputeRealZMP jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(ComputeRealZMP abstract-robot-dynamics)

ADD_EXECUTABLE(	ComputeTorqueOnTrajectory ComputeTorqueOnTrajectory.cpp)
target_link_libraries(ComputeTorqueOnTrajectory jrl-dynamics)
ADD_DEPENDENCIES (ComputeTorqueOnTrajectory jrl-dynamics)
PKG_CONFIG_USE_DEPENDENCY(ComputeTorqueOnTrajectory jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(ComputeTorqueOnTrajectory abstract-robot-dynamics)

ADD_EXECUTABLE(	TestFreeFlyer TestFreeFlyer.cpp CommonTools.cpp)
target_link_libraries(TestFreeFlyer jrl-dynamics)
ADD_DEPENDENCIES(TestFreeFlyer jrl-dynamics)
PKG_CONFIG_USE_DEPENDENCY(TestFreeFlyer jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(TestFreeFlyer abstract-robot-dynamics)

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/unitTesting/output.ascii.cmake
		 ${CMAKE_BINARY_DIR}/unitTesting/output.ascii )

ADD_TEST(TestHumanoidDynamicRobot TestHumanoidDynamicRobot "output.ascii")

ADD_TEST(TestAbstractInterface TestAbstractInterface)

ADD_TEST(TestTwoLinksModel TestTwoLinksModel)

 JRL_DYNAMICS_ADD_TEST(TestSpatial_doubleByv)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vCrossm)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vPlusv)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vPlusvector)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vectorPlusv)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vMoinsv)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vBydouble)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vCrossvector)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vEgalvector)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_dvPlusdv)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_dvPlusvector)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_vectorPlusdv)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_dvMoinsdv)
# JRL_DYNAMICS_ADD_TEST(TestSpatial_dvEgalvector)

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
  DESTINATION share/jrl-dynamics/examples/
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
  DESTINATION share/jrl-dynamics/examples/data
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

