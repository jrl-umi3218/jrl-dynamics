# Olivier Stasse, JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST



# Create the library "dynamicsJRLJapan" 

# Create list of source files
# ---------------------------
SET(libdyn_src
fileReader.cpp 
Joint.cpp 
Body.cpp 
Hand.cpp 
DynamicBody.cpp 
MultiBody.cpp 
DynamicMultiBody.cpp 
HumanoidDynamicMultiBody.cpp 
HumanoidSpecificities.cpp  
SpiritVRMLReader.cpp )


# Create list of headers files
#-----------------------------
SET(libdyn_headers 
../include/${PROJECT_NAME}/dynamics-config.h
../include/${PROJECT_NAME}/dynamicJRLJapan_API.h
../include/${PROJECT_NAME}/robotDynamicsImpl.h
)

FOREACH(srcfile ${libdyn_src})
  SET(newsrcfile "")
  STRING(REGEX REPLACE "cpp$" "h" newsrcfile ${srcfile})
  SET(libdyn_headers ../include/${PROJECT_NAME}/${newsrcfile} ${libdyn_headers})
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

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build SOT with shared libraries." ON)


# Specify compiling options
# -------------------------
SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
      		      PROPERTIES 
	              COMPILE_FLAGS  "-I${PROJECT_SOURCE_DIR}/include/ -I${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} ${${PROJECT_NAME}_CXXFLAGS}"
				  LINK_FLAGS ${${PROJECT_NAME}_LDFLAGS}
	             )

# Specify header files
SET(${PROJECT_NAME}_HEADERS ${libdyn_headers})

# Installation 
# -------------
INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

INSTALL(FILES ${${PROJECT_NAME}}
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

IF(UNIX)
ADD_CUSTOM_COMMAND( TARGET ${PROJECT_NAME_LIB}
            POST_BUILD
	    COMMAND ${CMAKE_COMMAND} -E copy ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}.so ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}-${PROJECT_VERSION}.so )


INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/src/lib${PROJECT_NAME}-${PROJECT_VERSION}.so
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)	
ENDIF(UNIX)

IF(WIN32)
	SET (LIBRARY_EXTENSION "dll" "lib")

	
	SET( CONFIGURATION_EXTENSION "")
	SET( CONFIGURATION_FOLDER "/Release")
#		IF( ${configuration} STREQUAL "Debug")
#			SET( CONFIGURATION_EXTENSION "d")
#		ENDIF()

	FOREACH (lib_extension ${LIBRARY_EXTENSION} ) 
		ADD_CUSTOM_COMMAND( TARGET ${PROJECT_NAME_LIB}
			POST_BUILD
		COMMAND 
		${CMAKE_COMMAND} -E copy 
			${${PROJECT_NAME}_BINARY_DIR}/src${CONFIGURATION_FOLDER}/${PROJECT_NAME}.${lib_extension}
			${${PROJECT_NAME}_BINARY_DIR}/lib/${PROJECT_NAME}-${PROJECT_VERSION}${CONFIGURATION_EXTENSION}.${lib_extension}
			)
			
		INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/lib/${PROJECT_NAME}-${PROJECT_VERSION}.${lib_extension}
				DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
					PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)	

	ENDFOREACH (lib_extension)



ENDIF(WIN32)

