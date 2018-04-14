#ifndef ANIMATIONRENDER
#define ANIMATIONRENDER

#include "videostream.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "pythoncaller.h"
#include "imageevaluation.h"
#include <std_msgs/String.h>


class AnimationRender
{
public:
    AnimationRender(ros::NodeHandle *nH);

    /*!
     * \brief start Start test procedure
     */
    void start();

    /*!
     * \brief mAutostart
     */
    bool mAutostart;

private:
    ros::NodeHandle *mpNodeHandle;

    ros::NodeHandle mGlobalNodeHandle;

    ros::Publisher  mPubTrackerControl;

    ros::Subscriber mSubControl;

    ros::Subscriber mSubResponse;

    PythonCaller *mPyCaller;

    VideoStream *mVideoStream;

    /*!
     * \brief controlCallback Callback of /animation_render/control topic
     * \param msg Command
     */
    void controlCallback(const std_msgs::String::ConstPtr &msg);

    void responseCallback(const std_msgs::String::ConstPtr &msg);

    /*!
     * \brief mCurrentTemplateImgID Number of current template in template image list.
     */
    int mCurrentTemplateImgID;

    /*!
     * \brief mCurrentBackgroundImgID Number of current background in background image list.
     */
    int mCurrentBackgroundImgID;

    /*!
     * \brief getTestParameters Load parameters.
     */
    void getTestParameters();

    /*!
     * \brief x_
     */
    double x_;
    double y_;
    double z_;

    double ax_;
    double ay_;
    double az_;

    // parameters
    int mFrames;
    int mFPS;

    int res_x_, res_y_;

    /*!
     * \brief mListLength Length of image lists
     */
    int mListLength;

    int template_min_width_;
    int template_min_height_;

    int background_min_width_;
    int background_min_height_;

    XmlRpc::XmlRpcValue mTemplateParameters;
    XmlRpc::XmlRpcValue mModelParameters;

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

    /*!
     * \brief mModelParameter1 Model parameter 1
     * Cylinder: length
     * Plane:    heigth
     */
    double mModelParameter1;

    /*!
     * \brief mModelParameter2 Model parameter 2
     * Cylinder: radius
     * Plane:    width
     */
    double mModelParameter2;

    /*!
     * \brief mModelParameter3 Model parameter 3
     * Cylinder: faces
     * Plane:    Not used
     */
    double mModelParameter3;

    /*!
     * \brief mResize
     */
    int mResize;

    ImageEvaluation mTemplateEvaluation;

    void setTemplateModelParameters();
    void renderControlPublish(std::string msg);
    void trackerControlPublish(std::string msg);

};

#endif // ANIMATIONRENDER

