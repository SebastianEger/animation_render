#include <ros/ros.h>
#include <animationrender.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "animation_render");
    ros::NodeHandle nH("~");
    AnimationRender tester(&nH);
    ros::spin();
    return 0;
}
