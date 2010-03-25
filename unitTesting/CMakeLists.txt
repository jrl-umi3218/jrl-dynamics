# Olivier Stasse, Francois Keith, JRL, CNRS/AIST
# Creation: 4/11/2008
# History:
#
# Copyright CNRS/AIST

INCLUDE(CTest)

# For the test, we include the same files as the src
SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_CXXFLAGS}")

# For the test, we link with the library corresponding to the src,  which is not installed yet
IF(UNIX)
  SET(${PROJECT_NAME}_test_CXXFLAGS "${${PROJECT_NAME}_test_CXXFLAGS} -DUNIX")
  SET(${PROJECT_NAME}_test_LDFLAGS "${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBDIR_RP}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBINCLUSION_KW}${PROJECT_NAME} ${${PROJECT_NAME}_src_LDFLAGS}")

  IF(CMAKE_BUILD_TYPE MATCHES "DEBUG")
    IF(CMAKE_COMPILER_IS_GNUCXX)
      # SET (${PROJECT_NAME}_test_LDFLAGS " ${${PROJECT_NAME}_test_LDFLAGS} -lgcov")
      SET (${PROJECT_NAME}_test_LDFLAGS " ${${PROJECT_NAME}_test_LDFLAGS}")
    ENDIF(CMAKE_COMPILER_IS_GNUCXX)
  ENDIF(CMAKE_BUILD_TYPE MATCHES "DEBUG")

ENDIF(UNIX)

IF(WIN32)
  SET(${PROJECT_NAME}_test_LDFLAGS "${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src  ${LIBINCLUSION_KW}${PROJECT_NAME}${LIB_EXT} ${${PROJECT_NAME}_src_LDFLAGS}")
ENDIF(WIN32)

MESSAGE(STATUS " unitTesting: ${UNITTESTING_CXXFLAGS}")

# Installing testing files.
MESSAGE(STATUS "${${PROJECT_NAME}_BINARY_DIR}/unitTesting/sample.wrl")

ADD_EXECUTABLE(	TestHumanoidDynamicRobot CommonTools.cpp TestHumanoidDynamicRobot.cpp TestHumanoidDynamicRobotDebugLayale.cpp)
SET_TARGET_PROPERTIES(TestHumanoidDynamicRobot
	              PROPERTIES
		      COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
		      DEPENDS ${${PROJECT_NAME}_BINARY_DIR}/unitTesting/sample.wrl
)
SET_TARGET_PROPERTIES(TestHumanoidDynamicRobotDebugLayale
	              PROPERTIES
		      COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
		      DEPENDS ${${PROJECT_NAME}_BINARY_DIR}/unitTesting/sample.wrl
)

ADD_CUSTOM_COMMAND(TARGET TestHumanoidDynamicRobot POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/sample.wrl 
  ${${PROJECT_NAME}_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/sampleSpecificities.xml
  ${${PROJECT_NAME}_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/sampleLinkJointRank.xml
  ${${PROJECT_NAME}_BINARY_DIR}/unitTesting
)

ADD_CUSTOM_COMMAND(TARGET TestHumanoidDynamicRobotDebugLayale POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/sample.wrl 
  ${${PROJECT_NAME}_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/sampleSpecificities.xml
  ${${PROJECT_NAME}_BINARY_DIR}/unitTesting
  COMMAND ${CMAKE_COMMAND} -E copy ${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/sampleLinkJointRank.xml
  ${${PROJECT_NAME}_BINARY_DIR}/unitTesting
)

ADD_DEPENDENCIES (TestHumanoidDynamicRobot ${PROJECT_NAME})
ADD_DEPENDENCIES (TestHumanoidDynamicRobotDebugLayale ${PROJECT_NAME})

ADD_EXECUTABLE(	TestCopyJointTreeThroughAbstractInterface 
  CommonTools.cpp 
  HumanoidCopy.cpp
  TestCopyJointTreeThroughAbstractInterface.cpp)
SET_TARGET_PROPERTIES( TestCopyJointTreeThroughAbstractInterface
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
                      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
ADD_DEPENDENCIES (TestCopyJointTreeThroughAbstractInterface ${PROJECT_NAME})

ADD_EXECUTABLE(	Test2Models Test2Models.cpp)
SET_TARGET_PROPERTIES(Test2Models
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
ADD_DEPENDENCIES (Test2Models ${PROJECT_NAME})

ADD_EXECUTABLE(	ComputeRealZMP ComputeRealZMP.cpp)
SET_TARGET_PROPERTIES(ComputeRealZMP
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
ADD_DEPENDENCIES (ComputeRealZMP ${PROJECT_NAME})

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/unitTesting/output.ascii.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/unitTesting/output.ascii )

ADD_TEST(TestHumanoidDynamicRobot TestHumanoidDynamicRobot "output.ascii")

ADD_TEST(TestCopyJointTreeThroughAbstractInterface TestCopyJointTreeThroughAbstractInterface)
