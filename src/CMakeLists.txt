# Olivier Stasse, Francois Keith, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST
#
# Please refers to file License.txt for details on the license.


# Create the library "dynamicsJRLJapan"

# Import jrl-mal flags.
INCLUDE_DIRECTORIES(${JRL_MAL_INCLUDEDIR})

# Import abstract-robot-dynamics flags.
INCLUDE_DIRECTORIES(${ABSTRACT_ROBOT_DYNAMICS_INCLUDEDIR})

# Make sure to find Debug.h
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src/private/)

# Make sure to find Debug.h
INCLUDE_DIRECTORIES(BEFORE ${CMAKE_BINARY_DIR}/src/private/)

# Create list of source files
# ---------------------------
SET(SOURCES
  private/Body.cpp
  private/DynamicBodyPrivate.cpp
  private/DynamicMultiBodyActuated.cpp
  private/DynamicMultiBodyAngularMomentum.cpp
  private/DynamicMultiBodyArticularJacobian.cpp
  private/DynamicMultiBodyCenterOfMass.cpp
  private/DynamicMultiBodyInertiaMatrix.cpp
  private/DynamicMultiBodyJointRank.cpp
  private/DynamicMultiBodyNewtonEuler.cpp
  private/DynamicMultiBodyNewtonEulerBackwardDynamics.cpp
  private/DynamicMultiBodyProperties.cpp
  private/DynamicMultiBodyTree.cpp
  private/DynMultiBodyPrivate.cpp
  private/Foot.cpp
  private/fileReader.cpp
  private/Hand.cpp
  private/HumanoidSpecificities.cpp
  private/HumDynMultiBodyPrivate.cpp
  private/JointAnchorPrivate.cpp
  private/JointFreeFlyerPrivate.cpp
  private/JointPrivate.cpp
  private/JointPrivateInterface.cpp
  private/JointPrivateNormalization.cpp
  private/JointPrivateSpatial.cpp
  private/JointRotationPrivate.cpp
  private/JointTranslationPrivate.cpp
  private/MultiBody.cpp
  private/Spatial.cpp
  private/SpiritVRMLReader.cpp
)

SET(SOURCES ${SOURCES}
  public/Joint.cpp
  public/DynamicBody.cpp
  public/dynamicsJRLJapanFactory.cpp
  public/GeometricData.cpp
)

# Create list of headers files
#-----------------------------
SET(jrl-walkgen_HEADERS
../include/jrl/dynamics/dll.hh
../include/jrl/dynamics/dynamicsfactory.hh
../include/jrl/dynamics/dynamicbody.hh
../include/jrl/dynamics/foot.hh
../include/jrl/dynamics/hand.hh
../include/jrl/dynamics/joint.hh
../include/jrl/dynamics/robotdynamicsimpl.hh
../include/jrl/dynamics/dynamicrobot.hh
../include/jrl/dynamics/humanoiddynamicrobot.hh
../include/jrl/dynamics/geometricdata.hh
)


# Add main target: dynamicsJRLJapan
# ---------------------------------
ADD_LIBRARY(jrl-dynamics SHARED
  ${SOURCES}
)

# Specify soname, version and rpath
# ---------------------------------
SET_TARGET_PROPERTIES(jrl-dynamics
      		      PROPERTIES
		      VERSION ${PROJECT_VERSION}
		      SOVERSION ${PROJECT_VERSION}
		      INSTALL_RPATH lib
		      INSTALL_RPATH_USE_LINK_PATH TRUE
		      )

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build jrl-dynamics with shared libraries." ON)


# Add local test

# SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/src/private/ ${${PROJECT_NAME}_CXXFLAGS}")
# SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_test_CXXFLAGS}")

#IF(UNIX)
#  SET(${PROJECT_NAME}_test_CXXFLAGS "${${PROJECT_NAME}_test_CXXFLAGS} -DUNIX")
#  SET(${PROJECT_NAME}_test_LDFLAGS
#    "${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBDIR_RP}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBINCLUSION_KW}${PROJECT_NAME} ${${PROJECT_NAME}_src_LDFLAGS}")
#ENDIF(UNIX)
