#
# (UnitTesting CMake code borrowed from OCE)
# The unittest framework uses GoogleTest 1.6.0 (aka GTest)
# The GTest source code is available in the /unittest/gtest-1.6.0 folder
#
SET(GTEST_ROOT ./gtest-1.6.0)
FIND_PACKAGE(GTest)
INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIR})

FILE(GLOB TEST_DIRECTORIES ./*_test gtest_sandbox)
SUBDIRS(${TEST_DIRECTORIES})
