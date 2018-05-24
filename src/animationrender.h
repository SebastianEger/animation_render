#ifndef ANIMATIONRENDER
#define ANIMATIONRENDER

#include "videostream.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "pythoncaller.h"
#include "templateevaluation.h"
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>

class AnimationRender
{
public:

    AnimationRender();
    /*!
     * \brief AnimationRender Constructor
     * \param nH
     */
    AnimationRender(ros::NodeHandle *nH);

    ~AnimationRender();

    /*!
     * \brief start Start test procedure, is called by <StartTest> command
     */
    void start();

    /*!
     * \brief mAutostart Start tests instantly
     */
    bool mAutostart;

private:
    bool downloadTemplateImage();
    void initNextVideo();

    /*!
     * \brief controlCallback Callback function of /animation_render/control topic
     * \param msg Command
     */
    void controlCallback(const std_msgs::String::ConstPtr &msg);

    /*!
     * \brief responseCallback Callback function of /animation_render/response topic
     * \param msg Response
     */
    void responseCallback(const std_msgs::String::ConstPtr &msg);

    /*!
     * \brief mpNodeHandle
     */
    ros::NodeHandle *mpNodeHandle;

    /*!
     * \brief mGlobalNodeHandle
     */
    ros::NodeHandle mGlobalNodeHandle;

    /*!
     * \brief mPubTrackerControl
     */
    ros::Publisher  mPubTrackerControl;

    /*!
     * \brief mSubControl
     */
    ros::Subscriber mSubControl;

    /*!
     * \brief mSubResponse
     */
    ros::Subscriber mSubResponse;

    /*!
     * \brief mpPyCaller Pointer to PythonCaller class
     */
    PythonCaller *mpPyCaller;

    /*!
     * \brief mVideoStream Pointer to VideoStream class
     */
    VideoStream *mpVideoStream;

    /*!
     * \brief mTemplateEvaluation Pointer to ImageEvaluation class
     */
    TemplateEvaluation *mpTemplateEvaluation;

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
     * \brief mInitPose Stores init pose
     */
    Pose mInitPose;

    VideoOptions mVideoOptions;

    /*!
     * \brief mListLength Length of image lists
     */
    int mListLength;

    int mBackgroundListLength;
    int mTemplateListLength;

    /*!
     * \brief mTemplateMinWidth Minimal width of template image
     */
    int mTemplateMinWidth;

    /*!
     * \brief mTemplateMinHeight Minimal height of template image
     */
    int mTemplateMinHeight;

    /*!
     * \brief mBackgroundMinWidth Minimal width of background image
     */
    int mBackgroundMinWidth;

    /*!
     * \brief mBackgroundMinHeight Minimal height of background image
     */
    int mBackgroundMinHeight;

    /*!
     * \brief mTemplateParameters Array of template parameters
     */
    XmlRpc::XmlRpcValue mTemplateParameters;

    /*!
     * \brief mModelParameters Array of model parameters
     */
    XmlRpc::XmlRpcValue mModelParameters;

    /*!
     * \brief mSkipRender Skip render video step
     */
    bool mSkipRender;

    /*!
     * \brief mSkipImgList Skip creating new image lists
     */
    bool mSkipImgList;

    /*!
     * \brief mSkipGroundTruth Skip extracting ground truth data
     */
    bool mSkipGroundTruth;

    /*!
     * \brief mSkipImageDownload Skip download new image from image list
     */
    bool mSkipImageDownload;

    bool mSkipTemplateDownload;
    bool mSkipBackgroundDownload;

    std::string mRenderFilename;

    std::string mTemplateFilename;

    std::string mPathVideos;
    std::string mPathTemplates;

    /*!
     * \brief mAnimation Animation name
     */
    std::string mAnimation;

    /*!
     * \brief mObject Object name, for example: Cylinder, Plane, (Cube)
     */
    std::string mObject;

    /*!
     * \brief mTemplateKeywords Keywords for template image search
     */
    std::string mTemplateKeywords;

    /*!
     * \brief mBackgroundKeywords Keywords for background image search
     */
    std::string mBackgroundKeywords;

    /*!
     * \brief mKey Flickr Api Key
     */
    std::string mKey;

    /*!
     * \brief mSecrectKey Flickr Api Secret Key
     */
    std::string mSecrectKey;

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

    /*!
     * \brief setTemplateModelParameters
     */
    void setTemplateModelParameters();

    /*!
     * \brief renderControlPublish
     * \param msg
     */
    void renderControlPublish(std::string msg);

    /*!
     * \brief trackerControlPublish
     * \param msg
     */
    void trackerControlPublish(std::string msg);

};

#endif // ANIMATIONRENDER

