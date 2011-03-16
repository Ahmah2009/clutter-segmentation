// Bring in gtest
#include <gtest/gtest.h>

using namespace std;

// Declare a test
TEST(TestSuite, testCase1)
{
    cout << "testCase1" << endl;
}

// Declare another test
TEST(TestSuite, testCase2)
{
    cout << "testCase2" << endl;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

