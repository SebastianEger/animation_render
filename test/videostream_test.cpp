#include <gtest/gtest.h>
#include <videostream.h>


TEST(Stream, openStream) {
    ros::NodeHandle nH;
    VideoStream vStream(nH);
    ASSERT_EQ(5, 5);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

