#include <gtest/gtest.h>

using namespace std;

TEST(LoadData, loadAngleData)
{
    cout << "testCase1" << endl;
}

TEST(TestSuite, testCase2)
{
    cout << "testCase2" << endl;
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

