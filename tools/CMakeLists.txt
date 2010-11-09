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

# For tools, we include the same files as the src
SET(${PROJECT_NAME}_test_CXXFLAGS "-I${PROJECT_SOURCE_DIR}/include/ ${${PROJECT_NAME}_CXXFLAGS}")

# For tools, we link with the library corresponding to the src,  which is not installed yet
IF(UNIX)
  SET(${PROJECT_NAME}_test_CXXFLAGS "${${PROJECT_NAME}_test_CXXFLAGS} -DUNIX")
  SET(${PROJECT_NAME}_test_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}")

  IF(CMAKE_BUILD_TYPE MATCHES "DEBUG")
    IF(CMAKE_COMPILER_IS_GNUCXX)
      SET (${PROJECT_NAME}_test_LDFLAGS " ${${PROJECT_NAME}_test_LDFLAGS} -lgcov")
    ENDIF(CMAKE_COMPILER_IS_GNUCXX)
  ENDIF(CMAKE_BUILD_TYPE MATCHES "DEBUG")

ENDIF(UNIX)

IF(WIN32)
  SET(${PROJECT_NAME}_test_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}")
ENDIF(WIN32)

MESSAGE(STATUS " unitTesting: ${UNITTESTING_CXXFLAGS}")

# Installing testing files.
MESSAGE(STATUS "${${PROJECT_NAME}_BINARY_DIR}/tools/sample.wrl")

# ReadOpenHRPFileAndExport 
ADD_EXECUTABLE(	ReadOpenHRPFileAndExport 
  GenerateRobotForAMELIF.cpp 
  ReadOpenHRPFileAndExport.cpp)

SET_TARGET_PROPERTIES(ReadOpenHRPFileAndExport
	              PROPERTIES
		      COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
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
		      COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
		      DEPENDS ${${PROJECT_NAME}_BINARY_DIR}/tools/sample.wrl
)
TARGET_LINK_LIBRARIES (CreateVRMLPoseFile ${PROJECT_NAME})
ADD_DEPENDENCIES (CreateVRMLPoseFile ${PROJECT_NAME})

# CreateMapleModel 
ADD_EXECUTABLE( CreateMapleModel
  GenerateRobotForMaple.cpp
  CreateMapleModel.cpp)

SET_TARGET_PROPERTIES(CreateMapleModel
	              PROPERTIES
		      COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
TARGET_LINK_LIBRARIES (CreateMapleModel ${PROJECT_NAME})
ADD_DEPENDENCIES (CreateMapleModel ${PROJECT_NAME})

# ReadPosFile 
ADD_EXECUTABLE(	ReadPosFile ReadPosFile.cpp)
SET_TARGET_PROPERTIES(ReadPosFile
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
TARGET_LINK_LIBRARIES (ReadPosFile ${PROJECT_NAME})
ADD_DEPENDENCIES (ReadPosFile ${PROJECT_NAME})


ADD_EXECUTABLE(CreateHPPBuilder 
  GenerateRobotForHppBuilder.cpp
  CreateHPPBuilder.cpp)

SET_TARGET_PROPERTIES(CreateHPPBuilder
	              PROPERTIES
	              COMPILE_FLAGS ${${PROJECT_NAME}_test_CXXFLAGS}
		      LINK_FLAGS ${${PROJECT_NAME}_test_LDFLAGS}
)
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
