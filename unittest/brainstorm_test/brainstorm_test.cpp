#include <gtest/gtest.h>

#ifndef UNITTEST_NO_HEEKS
#define UNITTEST_NO_HEEKS
#endif

#include "src/Contour.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <iostream>

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

TEST(ContourTestSuite, testSortEdges)
{
    /*
    Here's the stated goal of CContour::SortEdges(): We want consistency so
    that, if we use this Contour operation as a location for drilling a relief
    hole (one day), we want to be sure the machining will begin from a
    consistently known location.

    SortEdges() doesn't actually meet this goal! Here's a demonstration:
    
    We make a set of points in a rough circle; we connect the edges; we order
    the edges traversing in the natural order from East counterclockwise. Then
    we sort.

    If we reorder the edges (say, in alphabetical order) and then sort again,
    we should arrive at the same sort order.

    Below, the first sort yields a counterclockwise traversal. The second sort
    yields a clockwise traversal.
    */
    gp_Pnt
      pE(5,0,0),
      pENE(4,3,0),
      pNNE(3,4,0),
      pN(0,5,0),
      pNNW(-3,4,0),
      pWNW(-4,3,0),
      pW(-5,0,0),
      pWSW(-4,-3,0),
      pSSW(-3,-4,0),
      pS(0,-5,0),
      pSSE(3,-4,0),
      pESE(4,-3,0);
    TopoDS_Edge
      eENE(BRepBuilderAPI_MakeEdge(pE, pENE)),
      eNE(BRepBuilderAPI_MakeEdge(pENE, pNNE)),
      eNNE(BRepBuilderAPI_MakeEdge(pNNE, pN)),
      eNNW(BRepBuilderAPI_MakeEdge(pN, pNNW)),
      eNW(BRepBuilderAPI_MakeEdge(pNNW, pWNW)),
      eWNW(BRepBuilderAPI_MakeEdge(pWNW, pW)),
      eWSW(BRepBuilderAPI_MakeEdge(pW, pWSW)),
      eSW(BRepBuilderAPI_MakeEdge(pWSW, pSSW)),
      eSSW(BRepBuilderAPI_MakeEdge(pSSW, pS)),
      eSSE(BRepBuilderAPI_MakeEdge(pS, pSSE)),
      eSE(BRepBuilderAPI_MakeEdge(pSSE, pESE)),
      eESE(BRepBuilderAPI_MakeEdge(pESE, pE));

    BRepBuilderAPI_MakeWire wm1, wm2;

    /* Add edges in counterclockwise order. */
    wm1.Add(eENE);
    wm1.Add(eNE);
    wm1.Add(eNNE);
    wm1.Add(eNNW);
    wm1.Add(eNW);
    wm1.Add(eWNW);
    wm1.Add(eWSW);
    wm1.Add(eSW);
    wm1.Add(eSSW);
    wm1.Add(eSSE);
    wm1.Add(eSE);
    wm1.Add(eESE);
    vector<TopoDS_Edge> edges1(CContour::SortEdges(wm1.Wire()));

    /* Add edges in alphabetical order. */
    wm2.Add(eENE);
    wm2.Add(eESE);
    wm2.Add(eNE);
    wm2.Add(eNNE);
    wm2.Add(eNNW);
    wm2.Add(eNW);
    wm2.Add(eSE);
    wm2.Add(eSSE);
    wm2.Add(eSSW);
    wm2.Add(eSW);
    wm2.Add(eWNW);
    wm2.Add(eWSW);
    vector<TopoDS_Edge> edges2(CContour::SortEdges(wm2.Wire()));

    vector<TopoDS_Edge>::iterator i1, i2;

    /* i1 iterates counterclockwise. i2 iterates in the opposite direction!  */
    i1 = edges1.begin(), i2 = edges2.begin();
    EXPECT_EQ(*i1, *i2);
    i1++, i2++;
    for ( ; i1!=edges1.end() && i2!=edges2.end(); i1++, i2++)
    {
      EXPECT_NE(*i1, *i2);
    }

    /* These are just sanity checks that an ordering agrees with itself.  */
    i1 = edges1.begin(), i2 = edges1.begin();
    for (; i1!=edges1.end() && i2!=edges1.end(); i1++, i2++)
    {
      EXPECT_EQ(*i1, *i2);
    }
    i1 = edges2.begin(), i2 = edges2.begin();
    for (; i1!=edges2.end() && i2!=edges2.end(); i1++, i2++)
    {
      EXPECT_EQ(*i1, *i2);
    }
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

