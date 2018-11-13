#include <gtest/gtest.h>
#include <videostream.h>

int add(int a, int b) {
    return a+b;
}

TEST(VideoStream, IntegrationTest) {
    ASSERT_EQ(5, add(2, 4));
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "tester");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
