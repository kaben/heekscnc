#include <gtest/gtest.h>

#ifndef UNITTEST_NO_HEEKS
#define UNITTEST_NO_HEEKS
#endif

#include "src/Contour.h"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <algorithm>
#include <iostream>
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

template<typename _RandomAccessIterator>
inline void adaptive_merge_sort(
  _RandomAccessIterator __first,
  _RandomAccessIterator __last
){
  int length = __last - __first;
  if(length <= 1) return;
  _RandomAccessIterator __middle = __first + length / 2;
  adaptive_merge_sort(__first, __middle);
  adaptive_merge_sort(__middle, __last);
  if(*(__middle - 1) < *__middle) return;
  inplace_merge(__first, __middle, __last);
}

template<typename _RandomAccessIterator, typename _Compare>
inline void adaptive_merge_sort(
  _RandomAccessIterator __first,
  _RandomAccessIterator __last,
  _Compare __comp
){
  int length = __last - __first;
  if(length <= 1) return;
  _RandomAccessIterator __middle = __first + length / 2;
  adaptive_merge_sort(__first, __middle, __comp);
  adaptive_merge_sort(__middle, __last, __comp);
  if(__comp(*(__middle - 1), *__middle)) return;
  inplace_merge(__first, __middle, __last, __comp);
}

struct Comparison : public std::binary_function<const int &, const int &, bool >
{
  bool operator()(const int &l, const int &r) const { return l<r; }
};

TEST(MyAdaptivePartialSortTestSuite, brainstorm)
{
  vector<int> myints;
  myints.push_back(3);
  myints.push_back(7);
  myints.push_back(2);
  myints.push_back(5);
  myints.push_back(6);
  myints.push_back(4);
  myints.push_back(9);

  vector<int>::iterator it;
  int i = 0;

  it = myints.begin();
  i = 0;
  for (; it != myints.end(); it++) {
    cout << "i: " << ++i << endl;
    cout << "it: " << *it << endl;
  }
  cout << endl;

  adaptive_merge_sort(myints.begin(), myints.end(), Comparison());

  it = myints.begin();
  i = 0;
  for (; it != myints.end(); it++) {
    cout << "i: " << ++i << endl;
    cout << "it: " << *it << endl;
  }
  cout << endl;

  //blah_merge_sort(myints.begin(), myints.begin() + 1);
}

TEST(StdMinElementTestSuite, testIteration)
{
  vector<int> myints;
  myints.push_back(3);
  myints.push_back(7);
  myints.push_back(2);
  myints.push_back(5);
  myints.push_back(6);
  myints.push_back(4);
  myints.push_back(9);

  vector<int>::iterator min_it(min_element(myints.begin(), myints.end()));
  vector<int>::iterator it(min_it);

  cout << "found " << myints.size() << " ints." << endl << endl;
  int i = 0;
  while (true) {
    i++;
    cout << "bogus iteration: " << i << endl;
    cout << "element: " << *it << endl;
    it++;
    cout << "myints.begin(): " << *(myints.begin()) << endl;
    cout << "myints.end(): " << *(myints.end()) << endl;
    cout << "min_it): " << *min_it << endl;
    if (it == myints.end()) {
      cout << "wraparound!" << endl;
      it = myints.begin();
    }
    if (it == min_it) {
      cout << "breaking!" << endl;
      break;
    }
    cout << endl;
  }
  cout << "done." << endl;

  cout << "first true iteration." << endl;
  i = 0;
  it = myints.begin();
  for (; it != myints.end(); it++) {
    cout << "i: " << ++i << endl;
    cout << "it: " << *it << endl;
  }
  cout << "first true iteration done." << endl;

  cout << "second true iteration." << endl;
  i = 0;
  it = myints.begin();
  for (vector<int>::iterator it2 = myints.begin(); it != myints.end(); it++, it2++) {
    cout << "i: " << ++i << endl;
    //if (it2 == myints.begin()) {
    //  cout << "at begin..." << endl;
    //  partial_sort(myints.begin(), myints.begin()+1, myints.end());
    //} else {
    //  cout << "in middle..." << endl;
    //  //vector<int>::iterator it3(it2);
    //  //it3++;
    //  //if (it3 != myints.end()) {
    //  //  cout << "not at end. partial-sorting..." << endl;
    //  //  partial_sort(it3, it3+1, myints.end());
    //  //}
    //  partial_sort(it2, it2+1, myints.end());
    //}
    partial_sort(it2, it2+1, myints.end());
    cout << "it: " << *it << endl;
    cout << "it2: " << *it2 << endl;
  }
  cout << "second true iteration done." << endl;

  cout << "third true iteration." << endl;
  i = 0;
  it = myints.begin();
  for (; it != myints.end(); it++) {
    cout << "i: " << ++i << endl;
    cout << "it: " << *it << endl;
  }
  cout << "third true iteration done." << endl;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

