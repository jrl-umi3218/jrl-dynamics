# Olivier Stasse, Francois Keith, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST
#
# Please refers to file License.txt for details on the license.


# Create the library "dynamicsJRLJapan" 

# Create list of source files
# ---------------------------
SET(libdyn_src
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

SET(libdyn_src ${libdyn_src}
  public/Joint.cpp
  public/DynamicBody.cpp
  public/deprecated.cpp
  public/dynamicsJRLJapanFactory.cpp
  public/GeometricData.cpp
)

# Create list of headers files
#-----------------------------
SET(libdyn_headers 
../include/${PROJECT_NAME}/deprecated.h
../include/${PROJECT_NAME}/dll.h
../include/${PROJECT_NAME}/dynamicsJRLJapanFactory.h
../include/${PROJECT_NAME}/DynamicBody.h
../include/${PROJECT_NAME}/Foot.h
../include/${PROJECT_NAME}/Hand.h
../include/${PROJECT_NAME}/Joint.h 
../include/${PROJECT_NAME}/robotDynamicsImpl.h
../include/${PROJECT_NAME}/dynamicRobot.h
../include/${PROJECT_NAME}/humanoidDynamicRobot.h
../include/${PROJECT_NAME}/GeometricData.h
)

# Create name of the library.
#---------------------------
SET(PROJECT_NAME_LIB ${PROJECT_NAME})

# Add main target: dynamicsJRLJapan
# ---------------------------------
ADD_LIBRARY(${PROJECT_NAME_LIB}
  ${libdyn_src} ${libdyn_headers}
)

# Specify soname, version and rpath
# ---------------------------------
SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
      		      PROPERTIES 
		      VERSION ${PROJECT_VERSION}
		      SOVERSION ${PROJECT_VERSION}
		      INSTALL_RPATH lib
		      INSTALL_RPATH_USE_LINK_PATH TRUE
		      )

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build SOT with shared libraries." ON)


# Specify compiling options
# -------------------------
SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
      		      PROPERTIES 
	              COMPILE_FLAGS  "-I${PROJECT_SOURCE_DIR}/include -I${PROJECT_BINARY_DIR}/src/private -I${PROJECT_SOURCE_DIR}/src/private ${${PROJECT_NAME}_CXXFLAGS}"
		      LINK_FLAGS ${${PROJECT_NAME}_src_LDFLAGS}
	             )

# Specify header files
SET(${PROJECT_NAME}_HEADERS ${libdyn_headers})

# Installation 
# -------------
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
	DESTINATION include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

IF(UNIX)

INSTALL(FILES 
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so.${PROJECT_VERSION}
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so
	DESTINATION lib/
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )

ENDIF(UNIX)



IF(WIN32)
	SET (LIBRARY_EXTENSION "dll" "lib")

	FOREACH (lib_extension ${LIBRARY_EXTENSION} ) 
		ADD_CUSTOM_COMMAND( TARGET ${PROJECT_NAME_LIB}
			POST_BUILD
			COMMAND 
			${CMAKE_COMMAND} -E copy 
				${${PROJECT_NAME}_BINARY_DIR}/src/${PROJECT_NAME}.${lib_extension}
				${${PROJECT_NAME}_BINARY_DIR}/src/${PROJECT_NAME}-${PROJECT_VERSION}.${lib_extension}
		)
			
		INSTALL(FILES
			${${PROJECT_NAME}_BINARY_DIR}/src/${PROJECT_NAME}-${PROJECT_VERSION}.${lib_extension}
			${${PROJECT_NAME}_BINARY_DIR}/src/${PROJECT_NAME}.${lib_extension}
		
			DESTINATION lib
				PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
		)	

	ENDFOREACH (lib_extension)



ENDIF(WIN32)

# Add local test

SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/src/private/ ${${PROJECT_NAME}_CXXFLAGS}")
SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_test_CXXFLAGS}")

IF(UNIX)
  SET(${PROJECT_NAME}_test_CXXFLAGS "${${PROJECT_NAME}_test_CXXFLAGS} -DUNIX")
  SET(${PROJECT_NAME}_test_LDFLAGS "${LIBDIR_KW}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBDIR_RP}${${PROJECT_NAME}_BINARY_DIR}/src ${LIBINCLUSION_KW}${PROJECT_NAME} ${${PROJECT_NAME}_src_LDFLAGS}")

ENDIF(UNIX)

#ADD_EXECUTABLE(	TestHumanoidParser private/TestHumanoidParser.cpp)
#SET_TARGET_PROPERTIES(TestHumanoidParser
#	              PROPERTIES
#	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
#		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
#)

