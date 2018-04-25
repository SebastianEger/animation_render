#include "animationrender.h"

AnimationRender::AnimationRender()
{
    mpNodeHandle = new ros::NodeHandle("~");
}

AnimationRender::AnimationRender(ros::NodeHandle *nH) :
    mpNodeHandle(nH)
{
    mGlobalNodeHandle = ros::NodeHandle();

    mpPyCaller          = new PythonCaller(&mGlobalNodeHandle);
    mpVideoStream        = new VideoStream(mGlobalNodeHandle);
    mpTemplateEvaluation = new ImageEvaluation();

    // Set fps to default 25
    mpVideoStream->mFPS = 25;

    // Get control topic
    std::string controlTopic;
    mpNodeHandle->param<std::string>("control_topic", controlTopic, "/tplsearch/control");

    // Publish to given controlTopic, subscribe to /animation_render/control topic
    mPubTrackerControl = mGlobalNodeHandle.advertise<std_msgs::String>(controlTopic, 1);
    mSubControl        = mGlobalNodeHandle.subscribe<std_msgs::String>("/animation_render/control", 1, &AnimationRender::controlCallback, this);
    mSubResponse       = mGlobalNodeHandle.subscribe<std_msgs::String>("/animation_render/response", 1, &AnimationRender::responseCallback, this);

    // Set image ids
    mCurrentTemplateImgID   = 0;
    mCurrentBackgroundImgID = 0;

    // Get parameters
    getTestParameters();

    /*
    if(!mSkipGroundTruth) {
        std::string dataPath;
        mGlobalNodeHandle.param<std::string>("/tplsearch/data_path", dataPath, "~/tplsearch_data");
        mpPyCaller->getGroundTruthData(mAnimation, mFrames, dataPath + "/ground_truth");
    }
    */

    // Init template parameters
    mTemplateParameters    = XmlRpc::XmlRpcValue();
    mTemplateParameters[0] = XmlRpc::XmlRpcValue();

    // Init model parameters
    mModelParameters    = XmlRpc::XmlRpcValue();
    mModelParameters[0] = XmlRpc::XmlRpcValue();
}

AnimationRender::~AnimationRender()
{
    delete mpPyCaller;
    delete mpVideoStream;
    delete mpTemplateEvaluation;
}

void AnimationRender::start()
{
    mpPyCaller->getInitPose(mAnimation, mInitPose);
    if(!mSkipImgList) {
        mpPyCaller->getTemplateImageList(mListLength, mTemplateMinHeight, mTemplateMinWidth, mTemplateKeywords);
        mpPyCaller->getBackgroundImageList(mListLength, mBackgroundMinHeight, mBackgroundMinWidth, mBackgroundKeywords);
    }

    // Create template and model parameters
    setTemplateModelParameters();

    // Send Init command to tracker
    trackerControlPublish("Init");
}

void AnimationRender::controlCallback(const std_msgs::String::ConstPtr &msg)
{
    // COMMANDS
    if(msg->data == "StartTest") {
        start();
    }
    if(msg->data == "RenderVideo") {
        mpPyCaller->renderVideo(mFrames, mFPS, mObject, mAnimation, mWidth, mHeight, mModelParameter1, mModelParameter2, mModelParameter3);
    }
    if(msg->data == "CreateImageList") {
        mpPyCaller->getTemplateImageList(mListLength, mTemplateMinHeight, mTemplateMinWidth, mTemplateKeywords);
        mpPyCaller->getBackgroundImageList(mListLength, mBackgroundMinHeight, mBackgroundMinWidth, mBackgroundKeywords);
    }
    if(msg->data == "DownloadNextImage") {
        mpPyCaller->downloadTemplateImage(mCurrentTemplateImgID);
        mpPyCaller->downloadBackgroundImage(mCurrentBackgroundImgID);

        mCurrentBackgroundImgID ++;
        if(mCurrentBackgroundImgID == mListLength) {
            mCurrentTemplateImgID ++;
            mCurrentBackgroundImgID = 0;
        }
    }
    if(msg->data == "RunVideo") {
        mpVideoStream->mFPS = mFPS;
        mpVideoStream->openStream(ros::package::getPath("animation_render") + "/render/video.avi");
        trackerControlPublish("ExportData");
    }
    if(msg->data == "SetParameters") {
        setTemplateModelParameters();
    }
    if(msg->data == "ShowTemplate") {
        std::string path =  ros::package::getPath("animation_render") +  "/img/template_image.jpg";
        ImageEvaluation::showImage(path);
    }
    if(msg->data == "EvaluateTemplate") {
        std::string path =  ros::package::getPath("animation_render") +  "/img/template_image.jpg";
        mpTemplateEvaluation->evaluateTemplate(path);
    }
    if(msg->data == "ExportGroundTruth") {
        mpPyCaller->getGroundTruthData(mAnimation, mFrames,  ros::package::getPath("animation_render") + "/ground_truth");
    }
}

void AnimationRender::responseCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "InitOK") {
        if(!mSkipRender) mpPyCaller->renderVideo(mFrames, mFPS, mObject, mAnimation, mWidth, mHeight, mModelParameter1, mModelParameter2, mModelParameter3);
        mpVideoStream->mFPS = mFPS;
        mpVideoStream->openStream(ros::package::getPath("animation_render") + "/render/video.avi");
        trackerControlPublish("ExportData");
    }
    if(msg->data == "ExportDataOK") {
        mCurrentBackgroundImgID ++;
        if(mCurrentBackgroundImgID == mListLength) {
            mCurrentTemplateImgID ++;
            mCurrentBackgroundImgID = 0;
        }
        if(mCurrentTemplateImgID == mListLength) {
            trackerControlPublish("Shutdown");
            ros::shutdown();
            return;
        }
        if(!mSkipImageDownload) {
            mpPyCaller->downloadTemplateImage(mCurrentTemplateImgID);
            mpPyCaller->downloadBackgroundImage(mCurrentBackgroundImgID);
        }


        trackerControlPublish("Init");
    }
}

void AnimationRender::getTestParameters()
{
    mpNodeHandle->param<bool>("autostart", mAutostart, false);
    mpNodeHandle->param<int>("list_length", mListLength, 100);

    //
    mpNodeHandle->param<bool>("skip_render",         mSkipRender,        false);
    mpNodeHandle->param<bool>("skip_img_list",       mSkipImgList,       false);
    mpNodeHandle->param<bool>("skip_ground_truth",   mSkipGroundTruth,   true);
    mpNodeHandle->param<bool>("skip_image_download", mSkipImageDownload, false);

    // Video parameters
    mpNodeHandle->param<int>("frames",              mFrames, 100);
    mpNodeHandle->param<int>("fps",                 mFPS,    25);
    mpNodeHandle->param<int>("render_image_width",  mWidth,  600);
    mpNodeHandle->param<int>("render_image_height", mHeight, 600);

    mpNodeHandle->param<std::string>("animation",           mAnimation,          "TestAnimation");
    mpNodeHandle->param<std::string>("object",              mObject,             "Cylinder");
    mpNodeHandle->param<std::string>("template_keywords",   mTemplateKeywords,   "bird");
    mpNodeHandle->param<std::string>("background_keywords", mBackgroundKeywords, "plane");

    // Model parameters
    mpNodeHandle->param<double>("model_parameter_1", mModelParameter1, 0.05);
    mpNodeHandle->param<double>("model_parameter_2", mModelParameter2, 0.01);
    mpNodeHandle->param<double>("model_parameter_3", mModelParameter3, 200);

    mpNodeHandle->param<int>("template_min_height",   mTemplateMinHeight,   500);
    mpNodeHandle->param<int>("template_min_width",    mTemplateMinWidth,    500);
    mpNodeHandle->param<int>("background_min_height", mBackgroundMinHeight, 500);
    mpNodeHandle->param<int>("background_min_width",  mBackgroundMinWidth,  500);

    // Template parameters
    mpNodeHandle->param<int>("template_resize",  mResize,  100);
}

void AnimationRender::setTemplateModelParameters()
{
    ROS_INFO("Setting template and model parameters.");

    mTemplateParameters[0]["filename"] = ros::package::getPath("animation_render") + "/img/template_image.jpg";
    mTemplateParameters[0]["resize"]   = mResize;

    mModelParameters[0]["model_name"] = mObject;
    mModelParameters[0]["x"]      = mInitPose.x;
    mModelParameters[0]["y"]      = mInitPose.y;
    mModelParameters[0]["z"]      = mInitPose.z;
    mModelParameters[0]["ax"]     = mInitPose.ax;
    mModelParameters[0]["ay"]     = mInitPose.ay;
    mModelParameters[0]["az"]     = mInitPose.az;
    mModelParameters[0]["length"] = mModelParameter1;
    mModelParameters[0]["radius"] = mModelParameter2;
    mModelParameters[0]["height"] = mModelParameter1;
    mModelParameters[0]["width"]  = mModelParameter2;

    // set new parameters
    mGlobalNodeHandle.setParam("/tplsearch/templates", mTemplateParameters);
    mGlobalNodeHandle.setParam("/tplsearch/models",    mModelParameters);
}

void AnimationRender::trackerControlPublish(std::string msg)
{
    std_msgs::String toSend;
    toSend.data = msg;
    mPubTrackerControl.publish(toSend);
}
