#include <gtest/gtest.h>

#ifndef UNITTEST_NO_HEEKS
#define UNITTEST_NO_HEEKS
#endif

#include "src/Contour.h"
#include <TopoDS_Edge.hxx>
#include <functional>
#include <algorithm>
#include <vector>

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

struct EdgeComparison : public binary_function<const TopoDS_Edge &, const TopoDS_Edge &, bool >
{
    EdgeComparison( const TopoDS_Edge & edge )
    {
        m_reference_edge = edge;
    }

    bool operator()( const TopoDS_Edge & lhs, const TopoDS_Edge & rhs ) const
    {

        std::vector<double> lhs_distances;
        lhs_distances.push_back( CContour::GetStart(m_reference_edge).Distance( CContour::GetStart(lhs) ) );
        lhs_distances.push_back( CContour::GetStart(m_reference_edge).Distance( CContour::GetEnd(lhs) ) );
        lhs_distances.push_back( CContour::GetEnd(m_reference_edge).Distance( CContour::GetStart(lhs) ) );
        lhs_distances.push_back( CContour::GetEnd(m_reference_edge).Distance( CContour::GetEnd(lhs) ) );
        std::sort(lhs_distances.begin(), lhs_distances.end());

        std::vector<double> rhs_distances;
        rhs_distances.push_back( CContour::GetStart(m_reference_edge).Distance( CContour::GetStart(rhs) ) );
        rhs_distances.push_back( CContour::GetStart(m_reference_edge).Distance( CContour::GetEnd(rhs) ) );
        rhs_distances.push_back( CContour::GetEnd(m_reference_edge).Distance( CContour::GetStart(rhs) ) );
        rhs_distances.push_back( CContour::GetEnd(m_reference_edge).Distance( CContour::GetEnd(rhs) ) );
        std::sort(rhs_distances.begin(), rhs_distances.end());

        return(*(lhs_distances.begin()) < *(rhs_distances.begin()));
    }

    TopoDS_Edge m_reference_edge;
};


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

