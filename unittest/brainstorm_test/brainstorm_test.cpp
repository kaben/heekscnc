#include <gtest/gtest.h>

#ifndef UNITTEST_NO_HEEKS
#define UNITTEST_NO_HEEKS
#endif

#include "src/Contour.h"
#include <BRepBuilderAPI_MakeEdge.hxx>

using namespace std;


/*
This example demonstrates how to use gtest to conduct C++ unittesting
with CTest/CMake.
A TestSuite is by definition composed of several testCases.
Declare a first test:
  - TestSuite is CamelCase
  - testCase1 is camelCase
  
In the following, ASSERT_* is stronger than EXPECT_*
*/

TEST(TestSuite, testNullPointer)
{
    int *i=NULL;
    ASSERT_EQ(NULL,i);
}

TEST(TestSuite, testFloatEq)
{
    ASSERT_EQ(1.0, 1.0);
}

TEST(TestSuite, testFloatNeq)
{
    EXPECT_NE(.5, 1.);
}

TEST(TestSuite, testBoolean)
{
    bool b = true;
    EXPECT_TRUE(b);
}

TEST(TestSuite, testIntegerLighter)
{
    int a=1,b=2;
    EXPECT_LT(a,b);
}

TEST(EdgeComparisonTestSuite, testInstantiation)
{
    EdgeComparison x(BRepBuilderAPI_MakeEdge(gp_Pnt(0,0,0), gp_Pnt(1,0,0)));
}

TEST(EdgeComparisonTestSuite, testComparison)
{
    /*
    Three edges:
    - comparison edge O from (0,0,0) to (1,0,0)
    - edge A from ((0,1,0), (1,1,0))
    - edge B from ((0,2,0), (1,2,0))
    A should be closer to O than B.
    */
    gp_Pnt p000(0,0,0), p100(1,0,0), p010(0,1,0), p110(1,1,0), p020(0,2,0), p120(1,2,0);
    TopoDS_Edge O(BRepBuilderAPI_MakeEdge(p000, p100));
    TopoDS_Edge A(BRepBuilderAPI_MakeEdge(p010, p110));
    TopoDS_Edge B(BRepBuilderAPI_MakeEdge(p020, p120));
    EdgeComparison edge_comparison(O);
    /* True indicates A is closer to O than B. */
    EXPECT_TRUE(edge_comparison(A,B));
    /* False indicates B is not closer to O than A. */
    EXPECT_FALSE(edge_comparison(B,A));
    /* False indicates A is not closer to O than A. Which makes sense. */
    EXPECT_FALSE(edge_comparison(A,A));
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

