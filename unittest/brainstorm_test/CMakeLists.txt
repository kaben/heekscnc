#FILE(GLOB TEST_SOURCES *.cpp ../../src/Contour.cpp)
FILE(GLOB TEST_SOURCES *.cpp ../../src/ContourStatics.cpp)
SET(SOURCE_FILES ${TEST_SOURCES})
INCLUDE_DIRECTORIES("." ${HeeksCadDir})

#ADD_DEFINITIONS("-Wall -g -DUNICODE -DHEEKSCAD -DHAVE_LIMITS -DWXUSINGDLL -DHAVE_LIMITS -DTIXML_USE_STL `wx-config  --cflags` -DUNITTEST -DUNITTEST_NO_HEEKS")
ADD_EXECUTABLE(brainstorm_test ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(brainstorm_test gcov ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY} ${wxWidgets_LIBRARIES}  ${OpenCASCADE_LIBRARIES} TKBO)
ADD_TEST(brainstorm_test brainstorm_test)
