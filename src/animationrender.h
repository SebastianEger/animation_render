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
     * \brief Download current template image
     * \return
     */
    bool downloadTemplateImage();

    /*!
     * \brief Dwonload current background image
     * \return
     */
    bool downloadBackgroundImage();

private:
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

    /*!
     * \brief Video options
     */
    VideoOptions mVideoOptions;

    /*!
     * \brief mListLength Length of image lists
     */
    int mListLength;

    /*!
     * \brief mBackgroundListLength
     */
    int mBackgroundListLength;

    /*!
     * \brief mTemplateListLength
     */
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
     * \brief Path where videos are created
     */
    std::string mPathVideos;

    /*!
     * \brief Folder of the templates
     */
    std::string mTemplateFolder;

    /*!
     * \brief Folder of the backgrounds
     */
    std::string mBackgroundFolder;

    /*!
     * \brief Animation name
     */
    std::string mAnimation;

    /*!
     * \brief Object name, for example: Cylinder, Plane, (Cube)
     */
    std::string mObject;

    /*!
     * \brief Keywords for template image search
     */
    std::string mTemplateKeywords;

    /*!
     * \brief Keywords for background image search
     */
    std::string mBackgroundKeywords;

    /*!
     * \brief Flickr Api Key
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
     * \brief Size of the resized model
     */
    int mResize;

    /*!
     * \brief Range for random rotation distortion
     */
    double mInitRotationRange;

    /*!
     * \brief Range for random translation distortion
     */
    double mInitTranslationRange;

    /*!
     * \brief Use white background instead of background image
     */
    bool mUseWhiteBackground;

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
