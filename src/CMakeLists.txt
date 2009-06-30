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
fileReader.cpp 
Joint.cpp 
Body.cpp 
Hand.cpp 
Foot.cpp
DynamicBody.cpp 
MultiBody.cpp 
DynamicMultiBody.cpp 
HumanoidDynamicMultiBody.cpp 
HumanoidSpecificities.cpp  
SpiritVRMLReader.cpp 
dynamicsJRLJapanFactory.cpp
)


# Create list of headers files
#-----------------------------
SET(libdyn_headers 
../include/${PROJECT_NAME}/dynamicsJRLJapanFactory.h
../include/${PROJECT_NAME}/Body.h
../include/${PROJECT_NAME}/DynamicBody.h
../include/${PROJECT_NAME}/DynamicMultiBody.h
../include/${PROJECT_NAME}/Foot.h
../include/${PROJECT_NAME}/Hand.h
../include/${PROJECT_NAME}/Joint.h 
../include/${PROJECT_NAME}/MultiBody.h
../include/${PROJECT_NAME}/robotDynamicsImpl.h
)

FOREACH(srcfile ${libdyn_src})
  SET(newsrcfile "")
  STRING(REGEX REPLACE "cpp$" "h" newsrcfile ${srcfile})
#   MESSAGE(STATUS "newsrcfile: " ${newsrcfile})
ENDFOREACH(srcfile)

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
		      INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib
		      INSTALL_RPATH_USE_LINK_PATH TRUE
		      )

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build SOT with shared libraries." ON)


# Specify compiling options
# -------------------------
SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
      		      PROPERTIES 
	              COMPILE_FLAGS  "-I${PROJECT_SOURCE_DIR}/include/ -I${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} ${${PROJECT_NAME}_CXXFLAGS}"
				  LINK_FLAGS ${${PROJECT_NAME}_src_LDFLAGS}
	             )

# Specify header files
SET(${PROJECT_NAME}_HEADERS ${libdyn_headers})

# Installation 
# -------------
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

IF(UNIX)

INSTALL(FILES 
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so.${PROJECT_VERSION}
        ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
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
		
			DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
				PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
		)	

	ENDFOREACH (lib_extension)



ENDIF(WIN32)


