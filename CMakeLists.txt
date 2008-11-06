# Olivier Stasse, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST

cmake_minimum_required(VERSION 2.6)

enable_language(CXX)

# Debug mode for cmake.
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set version
SET(PROJECT_VERSION 1.9.99)

# name of the project
SET(PROJECT_NAME dynamicsJRLJapan)
SET(PROJECT_CONTACT olivier.stasse@aist.go.jp)
SET(PROJECT_DESCRIPTION "Implementation of Newton Euler algorithm")
project(${PROJECT_NAME})

SET(CMAKE_INSTALL_PREFIX $ENV{ROBOTPKG_PATH})

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build dynamicsJRLJapan with shared libraries." ON)

# Check for required packages
# -------------------------
INCLUDE(FindPkgConfig)
pkg_check_modules(MALJRL REQUIRED MatrixAbstractLayer>=1.4 )
pkg_check_modules(ROBOTDYNAMICS REQUIRED abstractRobotDynamics>=1.5.99 )

SET(PROJECT_REQUIREMENTS "MatrixAbstractLayer >= 1.4 abstractRobotDynamics >= 1.5.99")

MESSAGE(STATUS "${MALJRL_CFLAGS}")
MESSAGE(STATUS "${ROBOTDYNAMICS_CFLAGS}")

FOREACH(option_cxxflags ${MALJRL_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${option_cxxflags} ${${PROJECT_NAME}_CXXFLAGS}")
ENDFOREACH(option_cxxflags)

FOREACH(option_cxxflags ${ROBOTDYNAMICS_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${option_cxxflags} ${${PROJECT_NAME}_CXXFLAGS}")
ENDFOREACH(option_cxxflags)

FOREACH(option_ldflags ${MALJRL_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

FOREACH(option_ldflags ${ROBOTDYNAMICS_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

SET(${PROJECT_NAME}_LDFLAGS "-L${CMAKE_PREFIX_INSTALL}/lib -l${PROJECT_NAME}-${PROJECT_VERSION} ${${PROJECT_NAME}_LDFLAGS}")

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

# Install the file in the appropriate place.
SET(install_libdir_pkgconfig ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc )

INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${install_libdir_pkgconfig}	
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)






