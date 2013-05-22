# Olivier Stasse, Francois Keith, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST
#
# Please refers to file License.txt for details on the license.

# Make sure local headers are found.
INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/src/private/)
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

#--------------------------
# Create list of headers files
#-----------------------------
SET(jrl-dynamics_HEADERS
include/jrl/dynamics/dll.hh
include/jrl/dynamics/dynamicsfactory.hh
include/jrl/dynamics/dynamicbody.hh
include/jrl/dynamics/foot.hh
include/jrl/dynamics/hand.hh
include/jrl/dynamics/joint.hh
include/jrl/dynamics/robotdynamicsimpl.hh
include/jrl/dynamics/dynamicrobot.hh
include/jrl/dynamics/humanoiddynamicrobot.hh
include/jrl/dynamics/geometricdata.hh
)


ADD_LIBRARY(jrl-dynamics SHARED ${SOURCES})
SET_TARGET_PROPERTIES(jrl-dynamics PROPERTIES SOVERSION ${PROJECT_VERSION})
INSTALL(TARGETS jrl-dynamics DESTINATION ${CMAKE_INSTALL_LIBDIR})

PKG_CONFIG_USE_DEPENDENCY(jrl-dynamics jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(jrl-dynamics abstract-robot-dynamics)
