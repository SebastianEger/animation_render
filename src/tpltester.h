#ifndef TPLTESTER
#define TPLTESTER

#include "videostream.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "pythoncaller.h"
#include <std_msgs/String.h>


class TplTester
{
public:
    TplTester(ros::NodeHandle *nH);

    void start();

    bool autostart_;

private:
    ros::NodeHandle *mpNodeHandle;
    ros::NodeHandle mGlobalNodeHandle;
    ros::Publisher pubTrackerControl_, pubRenderControl_;
    ros::Subscriber subControl_;

    PythonCaller *pyCaller;

    VideoStream *vstream;

    void controlCallback(const std_msgs::String::ConstPtr &msg);

    int currentTemplateImgID_;
    int currentBackgroundImgID_;

    void getTestParameters();
    void getTplsearchParameters();

    // init location
    double x_, y_, z_, ax_, ay_, az_;

    // size
    double s1_, s2_;

    // parameters
    int frames_;
    int fps_;
    int cyl_faces_;
    int res_x_, res_y_;

    int listLength_;

    int template_min_width_;
    int template_min_height_;

    int background_min_width_;
    int background_min_height_;

    XmlRpc::XmlRpcValue templates;

    bool skipRender_;
    bool skipImgList_;
    bool skipGroundTruth_;
    bool skipImageDownload_;

    std::string animation_;
    std::string object_; // Cylinder, Plane, Cube
    std::string template_keywords_;
    std::string background_keywords_;
    std::string key_;
    std::string secret_key_;

    // Model parameters
    double mp1, mp2, mp3;

    void setTemplateSearchParameters();
    void renderControlPublish(std::string msg);
    void trackerControlPublish(std::string msg);

};

#endif // TPLTESTER

