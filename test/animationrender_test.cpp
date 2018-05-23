#include <gtest/gtest.h>
#include <animationrender.h>

#include <thread>

TEST(TestControlCommands, createImageList) {

    ros::NodeHandle nH;
    AnimationRender animationRender(&nH);

    ros::Publisher  mPubAnimationRenderControl = nH.advertise<std_msgs::String>("/animation_render/control", 1);
    std_msgs::String toSend;
    toSend.data = "CreateImageList";

    mPubAnimationRenderControl.publish(toSend);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "AnimationRenderTest");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{while(ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();
    ros::shutdown();
    return res;
}
