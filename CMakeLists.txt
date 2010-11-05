# Copyright 2008, 2009, 2010, 
#
# Francois Keith
# Florent Lamiraux
# Pierre Gergondet
# Oussama Kanoun
# Anthony Mallet
# Nicolas Mansard
# Olivier Stasse
#
# JRL/LAAS, CNRS/AIST
#
# This file is part of dynamicsJRLJapan.
# dynamicsJRLJapan is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# dynamicsJRLJapan is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with dynamicsJRLJapan.  If not, see <http://www.gnu.org/licenses/>.
#
# Creation: 30/10/2008
#


cmake_minimum_required(VERSION 2.6)

enable_language(CXX)

ENABLE_TESTING()

# Debug mode for cmake.
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set version
set(CPACK_PACKAGE_VERSION_MAJOR 1)
set(CPACK_PACKAGE_VERSION_MINOR 19)
set(CPACK_PACKAGE_VERSION_PATCH 1)

SET(PROJECT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}.99)

IF(WIN32)
  SET(LIBDIR_KW "/LIBPATH:")
  SET(LIBINCLUSION_KW "")
  SET(LIB_EXT ".lib")
ENDIF(WIN32)

IF(UNIX)
  SET(LIBDIR_KW "-L")
  SET(LIBDIR_RP "-Wl,-R")
  SET(LIBINCLUSION_KW "-l")
  SET(LIB_EXT "")
ENDIF(UNIX)

# name of the project
SET(PROJECT_NAME jrl-dynamics)
SET(PROJECT_CONTACT olivier.stasse@aist.go.jp)
SET(PROJECT_DESCRIPTION "Implementation of Newton Euler algorithm")
project(${PROJECT_NAME})

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build dynamicsJRLJapan with shared libraries." ON)

# Check for required packages
# ---------------------------
INCLUDE(FindPkgConfig)

SET(MAL_REQUIRED_VERSION 1.8.3)
SET(ABSTROBOTDYNAMICS_REQUIRED_VERSION 1.16)

pkg_check_modules(MALJRL REQUIRED jrl-mal>=${MAL_REQUIRED_VERSION})

pkg_check_modules(ROBOTDYNAMICS REQUIRED abstract-robot-dynamics>=${ABSTROBOTDYNAMICS_REQUIRED_VERSION})
EXEC_PROGRAM(${PKG_CONFIG_EXECUTABLE} ARGS  --variable=docdir abstractRobotDynamics
  OUTPUT_VARIABLE _abstractRobotDynamics_DOCDIR)

SET(PROJECT_REQUIREMENTS "jrl-mal >= ${MAL_REQUIRED_VERSION}, abstract-robot-dynamics >= ${ABSTROBOTDYNAMICS_REQUIRED_VERSION}")

MESSAGE(STATUS "${MALJRL_CFLAGS}")
MESSAGE(STATUS "${ROBOTDYNAMICS_CFLAGS}")

IF(UNIX)
SET(${PROJECT_NAME}_CXXFLAGS "-Wall ${${PROJECT_NAME}_CXXFLAGS}")
ENDIF(UNIX)
IF(WIN32)
SET(WIN32_CXXFLAGS "-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_DEPRECATE /wd4250 /wd4251 /wd4275 /wd4996")
SET(WIN32_CXXFLAGS "${WIN32_CXXFLAGS} /wd4355") # 'this' : used in base member initializer list
SET(${PROJECT_NAME}_CXXFLAGS "${${PROJECT_NAME}_CXXFLAGS} ${WIN32_CXXFLAGS}")
ENDIF(WIN32)

SET(${PROJECT_NAME}_export_CXXFLAGS "")
SET(${PROJECT_NAME}_export_LDFLAGS "")

FOREACH(option_cxxflags ${ROBOTDYNAMICS_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS " ${${PROJECT_NAME}_CXXFLAGS} ${option_cxxflags}")
  SET(${PROJECT_NAME}_export_CXXFLAGS " ${${PROJECT_NAME}_export_CXXFLAGS} ${option_cxxflags}")
ENDFOREACH(option_cxxflags)

FOREACH(option_cxxflags ${MALJRL_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${${PROJECT_NAME}_CXXFLAGS} ${option_cxxflags}")
  SET(${PROJECT_NAME}_export_CXXFLAGS " ${${PROJECT_NAME}_export_CXXFLAGS} ${option_cxxflags}")
ENDFOREACH(option_cxxflags)

FOREACH(option_ldflags ${ROBOTDYNAMICS_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

FOREACH(option_ldflags ${MALJRL_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

SET(${PROJECT_NAME}_src_LDFLAGS ${${PROJECT_NAME}_LDFLAGS})

SET(${PROJECT_NAME}_LDFLAGS "${LIBDIR_KW}${CMAKE_PREFIX_INSTALL}/lib  ${${PROJECT_NAME}_LDFLAGS}")

IF(UNIX)
  SET(${PROJECT_NAME}_LDFLAGS "${${PROJECT_NAME}_LDFLAGS} ${LIBINCLUSION_KW}${PROJECT_NAME}${LIB_EXT}")
ENDIF(UNIX)

IF(WIN32)
  SET(${PROJECT_NAME}_LDFLAGS "${${PROJECT_NAME}_LDFLAGS} ${LIBINCLUSION_KW}${PROJECT_NAME}${LIB_EXT}")
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

CONFIGURE_FILE(
	${${PROJECT_NAME}_SOURCE_DIR}/src/private/dynamicsGetLine.h.cmake
	${${PROJECT_NAME}_BINARY_DIR}/src/private/dynamicsGetLine.h
)

# Optimisation flags when building with RELEASE
#----------------------------------------------
IF(CMAKE_BUILD_TYPE MATCHES "RELEASE")
  IF(CMAKE_COMPILER_IS_GNUCXX)
    SET (${PROJECT_NAME}_CXXFLAGS "-O3 -funroll-loops -frerun-loop-opt -fschedule-insns2 -frerun-cse-after-loop -falign-functions -falign-labels -falign-loops -falign-jumps -fexpensive-optimizations ${${PROJECT_NAME}_CXXFLAGS}")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDIF(CMAKE_BUILD_TYPE MATCHES "RELEASE")

# Profiling flags when building with DEBUG
#-----------------------------------------
IF(CMAKE_BUILD_TYPE MATCHES "DEBUG")
  IF(CMAKE_COMPILER_IS_GNUCXX)
    # SET (${PROJECT_NAME}_CXXFLAGS "-fprofile-arcs -ftest-coverage ${${PROJECT_NAME}_CXXFLAGS}")
    SET (${PROJECT_NAME}_CXXFLAGS "${${PROJECT_NAME}_CXXFLAGS}")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDIF(CMAKE_BUILD_TYPE MATCHES "DEBUG")
# To deal with the headers.
#--------------------------
#add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(unitTesting)
add_subdirectory(tools)

# Prepare the documentation
# -------------------------
OPTION(GENERATE_DOC "Generate html documentation" 1)
IF (GENERATE_DOC)
  INCLUDE(FindDoxygen)
  IF(DOXYGEN_FOUND)
    MESSAGE(STATUS "Doxygen found")
	ADD_SUBDIRECTORY(doc)
  ENDIF(DOXYGEN_FOUND)
ENDIF (GENERATE_DOC)

# Prepare description file for pkg-config.
#-----------------------------------------

SET(install_pkg_prefix "\${prefix}")
SET(install_pkg_exec_prefix "\${exec_prefix}")
SET(install_pkg_libdir "\${libdir}")
SET(install_pkg_include_dir "\${includedir}")
SET(install_pkg_datarootdir "\${datarootdir}")

# Install the file in the appropriate place.
SET(install_libdir_pkgconfig lib/pkgconfig)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc )

INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${install_libdir_pkgconfig}	
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)

# Uninstall target
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

#---------------
# Package target
#---------------
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "jrl-dynamics - Provides an implementation of the Recursive Newton-Euler Algorithm (RNEA).")
set(CPACK_PACKAGE_VENDOR "JRL CNRS/AIST")
set(CPACK_PACKAGE_DESCRIPTION_FILE ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
set(CPACK_PACKAGE_CONTACT "Olivier Stasse (olivier.stasse@aist.go.jp)")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "jrl-mal (>=1.8.3), abstractrobotdynamics (>=1.16)")
set(
CPACK_SOURCE_PACKAGE_FILE_NAME
"jrl-dynamics-src-${PROJECT_VERSION}"
CACHE INTERNAL "tarball basename"
)
set(CPACK_PACKAGE_NAME "jrl-dynamics")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_GENERATOR "DEB;TGZ")

# The following components are regex's to match anywhere (unless anchored)
# in absolute path + filename to find files or directories to be excluded
# from source tarball.
set(CPACK_SOURCE_IGNORE_FILES
"~$"
"^${PROJECT_SOURCE_DIR}/build/"
"^${PROJECT_SOURCE_DIR}/.git/"
)

set(CPACK_PACKAGING_INSTALL_PREFIX "/usr/openrobots")
#message("CPACK_SOURCE_IGNORE_FILES = ${CPACK_SOURCE_IGNORE_FILES}")
include(CPack)

