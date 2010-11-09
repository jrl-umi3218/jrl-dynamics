# Copyright 2010, 
#
# Francois Keith
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

# Import jrl-mal flags.
INCLUDE_DIRECTORIES(${JRL_MAL_INCLUDEDIR})

# Import abstract-robot-dynamics flags.
INCLUDE_DIRECTORIES(${ABSTRACT_ROBOT_DYNAMICS_INCLUDEDIR})

# For tools, we include the same files as the src
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include/)

#SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_CXXFLAGS}")

# Installing testing files.
MESSAGE(STATUS "${${PROJECT_NAME}_BINARY_DIR}/tools/sample.wrl")

# ReadOpenHRPFileAndExport 
ADD_EXECUTABLE(	ReadOpenHRPFileAndExport 
  GenerateRobotForAMELIF.cpp 
  ReadOpenHRPFileAndExport.cpp)

SET_TARGET_PROPERTIES(ReadOpenHRPFileAndExport
	              PROPERTIES
		      DEPENDS ${${PROJECT_NAME}_BINARY_DIR}/tools/sample.wrl
)
TARGET_LINK_LIBRARIES (ReadOpenHRPFileAndExport ${PROJECT_NAME})
ADD_DEPENDENCIES (ReadOpenHRPFileAndExport ${PROJECT_NAME})

# CreateVRMLPoseFile 
ADD_EXECUTABLE(	CreateVRMLPoseFile
  GenerateRobotForVRML2.cpp
  CreateVRMLPoseFile.cpp)

SET_TARGET_PROPERTIES(CreateVRMLPoseFile
	              PROPERTIES
		      DEPENDS ${${PROJECT_NAME}_BINARY_DIR}/tools/sample.wrl
)
TARGET_LINK_LIBRARIES (CreateVRMLPoseFile ${PROJECT_NAME})
ADD_DEPENDENCIES (CreateVRMLPoseFile ${PROJECT_NAME})

# CreateMapleModel 
ADD_EXECUTABLE( CreateMapleModel
  GenerateRobotForMaple.cpp
  CreateMapleModel.cpp)

TARGET_LINK_LIBRARIES (CreateMapleModel ${PROJECT_NAME})
ADD_DEPENDENCIES (CreateMapleModel ${PROJECT_NAME})

# ReadPosFile 
ADD_EXECUTABLE(	ReadPosFile ReadPosFile.cpp)
TARGET_LINK_LIBRARIES (ReadPosFile ${PROJECT_NAME})
ADD_DEPENDENCIES (ReadPosFile ${PROJECT_NAME})


ADD_EXECUTABLE(CreateHPPBuilder 
  GenerateRobotForHppBuilder.cpp
  CreateHPPBuilder.cpp)

TARGET_LINK_LIBRARIES (CreateHPPBuilder ${PROJECT_NAME})
ADD_DEPENDENCIES (CreateHPPBuilder ${PROJECT_NAME})


SET(${PROJECT_NAME}_TOOLS
  ReadOpenHRPFileAndExport
  CreateVRMLPoseFile
  CreateHPPBuilder
  CreateMapleModel
  ReadPosFile
)
INSTALL(TARGETS ${${PROJECT_NAME}_TOOLS}
  DESTINATION bin/
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE )
