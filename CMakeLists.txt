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

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

# name of the project
SET(PROJECT_NAME jrl-dynamics)
SET(PROJECT_CONTACT olivier.stasse@aist.go.jp)
SET(PROJECT_DESCRIPTION "Implementation of Newton Euler algorithm")
SET(PROJECT_URL "git@github.com:jrl-umi3218/jrl-dynamics.git")

SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE ON)

# To deal with the headers.
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

SETUP_PROJECT()

# Dependencies
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.9.0")
ADD_REQUIRED_DEPENDENCY("abstract-robot-dynamics >= 1.16")
#SEARCH_FOR_EIGEN()
#Set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -I/usr/local/include/eigen3)

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
	${CMAKE_SOURCE_DIR}/src/private/dynamicsGetLine.h.cmake
	${CMAKE_BINARY_DIR}/src/private/dynamicsGetLine.h
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

# Add dependency towards the library in the pkg-config file.
PKG_CONFIG_APPEND_LIBS("jrl-dynamics")

#add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(unitTesting)
add_subdirectory(tools)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
