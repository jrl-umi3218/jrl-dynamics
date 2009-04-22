# Olivier Stasse, Francois Keith, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST

cmake_minimum_required(VERSION 2.6)

enable_language(CXX)

# Debug mode for cmake.
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set version
SET(PROJECT_VERSION 1.13.1.99)

IF(WIN32)
  SET(LIBDIR_KW "/LIBPATH:")
  SET(LIBINCLUSION_KW "")
  SET(LIB_EXT ".lib")
ENDIF(WIN32)

IF(UNIX)
  SET(LIBDIR_KW "-L")
  SET(LIBINCLUSION_KW "-l")
  SET(LIB_EXT "")
ENDIF(UNIX)

# name of the project
SET(PROJECT_NAME dynamicsJRLJapan)
SET(PROJECT_CONTACT olivier.stasse@aist.go.jp)
SET(PROJECT_DESCRIPTION "Implementation of Newton Euler algorithm")
project(${PROJECT_NAME})

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build dynamicsJRLJapan with shared libraries." ON)

# Check for required packages
# ---------------------------
INCLUDE(FindPkgConfig)

SET(MAL_REQUIRED_VERSION 1.7.2)
SET(ABSTROBOTDYNAMICS_REQUIRED_VERSION 1.13)

pkg_check_modules(MALJRL REQUIRED MatrixAbstractLayer>=${MAL_REQUIRED_VERSION})
pkg_check_modules(ROBOTDYNAMICS REQUIRED abstractRobotDynamics>=${ABSTROBOTDYNAMICS_REQUIRED_VERSION})
EXEC_PROGRAM(${PKG_CONFIG_EXECUTABLE} ARGS  --variable=docdir abstractRobotDynamics
  OUTPUT_VARIABLE _abstractRobotDynamics_DOCDIR)

SET(PROJECT_REQUIREMENTS "MatrixAbstractLayer >= ${MAL_REQUIRED_VERSION}, abstractRobotDynamics >= ${ABSTROBOTDYNAMICS_REQUIRED_VERSION}")

MESSAGE(STATUS "${MALJRL_CFLAGS}")
MESSAGE(STATUS "${ROBOTDYNAMICS_CFLAGS}")

SET(${PROJECT_NAME}_CXXFLAGS "-Wall ${${PROJECT_NAME}_CXXFLAGS}")

FOREACH(option_cxxflags ${MALJRL_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${${PROJECT_NAME}_CXXFLAGS} ${option_cxxflags}")
ENDFOREACH(option_cxxflags)

FOREACH(option_cxxflags ${ROBOTDYNAMICS_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS " ${${PROJECT_NAME}_CXXFLAGS} ${option_cxxflags}")
ENDFOREACH(option_cxxflags)

FOREACH(option_ldflags ${MALJRL_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

FOREACH(option_ldflags ${ROBOTDYNAMICS_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

SET(${PROJECT_NAME}_src_LDFLAGS ${${PROJECT_NAME}_LDFLAGS})

SET(${PROJECT_NAME}_LDFLAGS "${LIBDIR_KW}${CMAKE_PREFIX_INSTALL}/lib  ${${PROJECT_NAME}_LDFLAGS}")

IF(UNIX)
  SET(${PROJECT_NAME}_LDFLAGS "${${PROJECT_NAME}_LDFLAGS} ${LIBINCLUSION_KW}${PROJECT_NAME}${LIB_EXT}")
ENDIF(UNIX)

IF(WIN32)
  SET(${PROJECT_NAME}_LDFLAGS "${${PROJECT_NAME}_LDFLAGS} ${LIBINCLUSION_KW}${PROJECT_NAME}-${PROJECT_VERSION}${LIB_EXT}")
ENDIF(WIN32)

# Check if glut is present or not.
# -------------------------------
INCLUDE(FindGLUT)			  

INCLUDE(CheckFunctionExists)
CHECK_FUNCTION_EXISTS("getline" HAVE_GETLINE)

IF(HAVE_GETLINE_PTR)
  MESSAGE(STATUS "We have getline")
  SET(HAVE_GETLINE_INT 1)
ELSE(HAVE_GETLINE_PTR)
  MESSAGE(STATUS " We don't have getline")
  SET(HAVE_GETLINE_INT 0)
ENDIF(HAVE_GETLINE_PTR)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/dynamics-config.h.cmake
	       ${${PROJECT_NAME}_SOURCE_DIR}/include/${PROJECT_NAME}/dynamics-config.h )

# To deal with the headers.
#--------------------------
#add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(unitTesting)

# Prepare the documentation
# -------------------------
INCLUDE(FindDoxygen)
IF(DOXYGEN_FOUND)
  MESSAGE(STATUS "Doxygen found")
  add_subdirectory(doc)
ENDIF(DOXYGEN_FOUND)

# Prepare description file for pkg-config.
#-----------------------------------------

SET(install_pkg_prefix "\${prefix}")
SET(install_pkg_exec_prefix "\${exec_prefix}")
SET(install_pkg_libdir "\${libdir}")
SET(install_pkg_include_dir "\${includedir}")
SET(install_pkg_datarootdir "\${datarootdir}")

# Install the file in the appropriate place.
SET(install_libdir_pkgconfig ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc )

INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${install_libdir_pkgconfig}	
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)






