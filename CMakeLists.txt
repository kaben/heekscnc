project( HeeksCNC )
cmake_minimum_required( VERSION 2.6 )
if( ${CMAKE_VERSION} STREQUAL "2.8.2" )
  # bug http://vtk.org/Bug/view.php?id=11020
  message( WARNING "CMake/CPack version 2.8.2 will not create working .deb packages!")
endif( ${CMAKE_VERSION} STREQUAL "2.8.2" )

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set( BUILD_STARTED_IN_RIGHT_DIR "true" )  #used in src/CMakeLists.txt to print a message

add_definitions ( -Wall -DOP_SKETCHES_AS_CHILDREN  )

find_package( wxWidgets REQUIRED COMPONENTS base core gl )
find_package( PythonInterp REQUIRED )

#find OCE or OpenCASCADE
set( CASCADE_LIBS TKernel )
if( UNIX )
  set( OCE_DIR "/usr/local/share/cmake/" )
else()
  set( OCE_DIR "c:/OCE-0.3.0/share/cmake" )
endif()
find_package ( OCE )
if( ${OCE_FOUND} )
  add_definitions ( -DHAVE_CONFIG_H )
  set( OpenCASCADE_LIBRARIES ${CASCADE_LIBS} )
  set( OpenCASCADE_INCLUDE_DIR ${OCE_INCLUDE_DIRS} )
else() #look for OpenCASCADE
  set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/" )
  find_package ( OpenCASCADE REQUIRED COMPONENTS ${CASCADE_LIBS} )
endif()

#find libarea
#SET(AREA_ROOT ./libarea/build)
#FIND_PACKAGE(AREA)
add_subdirectory(libarea)

include(${wxWidgets_USE_FILE})
include_directories (
	${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}
	${wxWidgets_INCLUDE_DIRS}
	${OpenCASCADE_INCLUDE_DIR}
	)

file( GLOB hcnc_py "${CMAKE_CURRENT_SOURCE_DIR}/*.py" "${CMAKE_CURRENT_SOURCE_DIR}/pycnc/*.py")
list( REMOVE_ITEM hcnc_py
                  "${CMAKE_CURRENT_SOURCE_DIR}/post.py"
                  "${CMAKE_CURRENT_SOURCE_DIR}/POST_TEST.py"
                  "${CMAKE_CURRENT_SOURCE_DIR}/STLTools.py"   )
install( FILES ${hcnc_py} DESTINATION lib/heekscnc )


IF( CMAKE_SIZEOF_VOID_P EQUAL 4 )
  set(PKG_ARCH i386)
ELSE( CMAKE_SIZEOF_VOID_P EQUAL 4 )
  set(PKG_ARCH amd64)
ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 4 )

#-------------------- package generation -------------------------
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "HeeksCAD Developers <heekscad-devel@googlegroups.com>")
set(CPACK_PACKAGE_NAME "heekscnc")
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 18)
set(CPACK_PACKAGE_VERSION_PATCH 0)
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_beta-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}_${PKG_ARCH}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HeeksCNC, CNC machining add-in for HeeksCAD
 HeeksCNC is a CNC machining (CAM) add-in for HeeksCAD. It produces
 g-code for CNC machines, such as those run by linuxcnc.org's EMC2")
set(CPACK_DEBIAN_PACKAGE_SECTION "science")
set(CPACK_DEBIAN_PACKAGE_VERSION "")

set(CPACK_DEBIAN_PACKAGE_DEPENDS "python-wxgtk2.8, libgtkglext1 (>= 1.2), python (>= 2.5), libboost-python1.40.0 | libboost-python1.42.0, python" )
set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "heekscad (>= 0.15.1), libopencascade-ocaf-6.3.0")


#--------------- these are down here so that the package version vars above are visible -------------
add_subdirectory( src )
set_directory_properties( PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CPACK_PACKAGE_FILE_NAME}.deb" )

#------------- include(CPack) should be the last line in this file
include(CPack)

############################################
# UnitTesting CMake code borrowed from OCE #
# See https://github.com/tpaviot/oce       #
############################################
# Define helper macro OPTION_WITH_DEFAULT
MACRO( OPTION_WITH_DEFAULT OPTION_NAME OPTION_STRING OPTION_DEFAULT )
	IF( NOT DEFINED ${OPTION_NAME} )
		SET( ${OPTION_NAME} ${OPTION_DEFAULT} )
	ENDIF( NOT DEFINED ${OPTION_NAME} )
	OPTION( ${OPTION_NAME} "${OPTION_STRING}" ${${OPTION_NAME}} )
ENDMACRO( OPTION_WITH_DEFAULT OPTION_NAME OPTION_STRING OPTION_DEFAULT )
OPTION_WITH_DEFAULT( ${PROJECT_NAME}_TESTING "Enable unittesting framework" OFF )
MARK_AS_ADVANCED(${PROJECT_NAME}_TESTING)
IF(${PROJECT_NAME}_TESTING)
	SET( ${PROJECT_NAME}_EXTRA_WARNINGS ON CACHE BOOL "Extra warnings required by testing framework" FORCE)
	INCLUDE(CTest)
	ENABLE_TESTING()
	SUBDIRS(unittest)
ENDIF(${PROJECT_NAME}_TESTING)
IF(${PROJECT_NAME}_COVERAGE)
	SET(${PROJECT_NAME}_TESTING ON CACHE BOOL "Testing enabled by coverage option" FORCE)
	# build static libs, better coverage report
	SET( ${PROJECT_NAME}_BUILD_SHARED_LIB OFF CACHE BOOL "Build static libs" FORCE )
	SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -fprofile-arcs -ftest-coverage" CACHE STRING "Extra compile flags required by code coverage" FORCE)
	SET(CMAKE_C_FLAGS_DEBUG "-O0 -g -fprofile-arcs -ftest-coverage" CACHE STRING "Extra compile flags required by code coverage" FORCE)
	SET(CMAKE_MODULE_LINKER_FLAGS_DEBUG "-fprofile-arcs -ftest-coverage" CACHE STRING "Extra linker flags required by code coverage" FORCE)
	SET(${PROJECT_NAME}_BUILD_TYPE "Debug" CACHE STRING "Build type required by testing framework" FORCE)
ENDIF(${PROJECT_NAME}_COVERAGE)

