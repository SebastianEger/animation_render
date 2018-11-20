#include "animationrender.h"
#include <random>

AnimationRender::AnimationRender()
{
    mpNodeHandle = new ros::NodeHandle("~");
}

AnimationRender::AnimationRender(ros::NodeHandle *nH) :
    mpNodeHandle(nH)
{
    mGlobalNodeHandle = ros::NodeHandle();

    mpPyCaller           = new PythonCaller(&mGlobalNodeHandle);
    mpVideoStream        = new VideoStream(mGlobalNodeHandle);
    mpTemplateEvaluation = new TemplateEvaluation();

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

    std::string videoFolderPath = mPathVideos    + "/" + mAnimation;

    boost::filesystem::path videoDir(videoFolderPath.c_str());
    boost::filesystem::path templateDir(mTemplateFolder.c_str());
    boost::filesystem::path backgroundDir(mBackgroundFolder.c_str());

    boost::filesystem::create_directories(videoDir);
    boost::filesystem::create_directories(templateDir);
    boost::filesystem::create_directories(backgroundDir);

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

bool AnimationRender::downloadTemplateImage()
{
    if(mCurrentTemplateImgID == mTemplateListLength) {
        return false;
    }
    std::string templateFileName = mTemplateFolder + "/template_" + std::to_string(mCurrentTemplateImgID);
    mpPyCaller->downloadImage(templateFileName, mTemplateFolder + "/list.txt", mCurrentTemplateImgID);
    return true;
}

bool AnimationRender::downloadBackgroundImage()
{
    if(mCurrentBackgroundImgID == mBackgroundListLength) {
        return false;
    }
    std::string backgroundFileName = mBackgroundFolder + "/background_" + std::to_string(mCurrentBackgroundImgID);
    mpPyCaller->downloadImage(backgroundFileName, mBackgroundFolder + "/list.txt", mCurrentBackgroundImgID);
    return true;
}

void AnimationRender::start()
{
    mpPyCaller->getInitPose(mAnimation, mInitPose);
    // Init img ids
    mCurrentTemplateImgID   = -1;
    mCurrentBackgroundImgID = 0;
    initNextVideo();
}

void AnimationRender::initNextVideo()
{
    mCurrentTemplateImgID++;
    if(mCurrentTemplateImgID == mTemplateListLength) {
        mCurrentTemplateImgID = 0;
        mCurrentBackgroundImgID++;
        if(mCurrentBackgroundImgID == mBackgroundListLength) {
            return;
        }
    }
    std::string filename = mTemplateFolder + "/template_" + std::to_string(mCurrentTemplateImgID);
    // check if template exists
    if(boost::filesystem::exists(filename + ".jpg")) {
        if(mpTemplateEvaluation->evaluate(filename + ".jpg")) {
            setTemplateModelParameters();
            // Send Init command to tracker
            trackerControlPublish("Init");
        } else {
            initNextVideo();
        }
    } else {
        initNextVideo();
    }
}

void AnimationRender::controlCallback(const std_msgs::String::ConstPtr &msg)
{
    // COMMANDS
    if(msg->data == "StartTest") {
        start();
    }
    if(msg->data == "RenderVideos") {
        mCurrentTemplateImgID   = 0;
        mCurrentBackgroundImgID = 0;

        while(true) {
            std::string video_name = mPathVideos + "/" + mAnimation + "/" + mObject + "/video_" + std::to_string(mCurrentTemplateImgID) + "_" + std::to_string(mCurrentBackgroundImgID);
            std::string template_name =   mTemplateFolder   + "/template_" + std::to_string(mCurrentTemplateImgID);
            std::string background_name = mBackgroundFolder + "/background_" + std::to_string(mCurrentBackgroundImgID);

            if(mUseWhiteBackground) background_name = "White";
            mpPyCaller->renderVideo(video_name, template_name, background_name, mVideoOptions, mObject, mAnimation, mModelParameter1, mModelParameter2, mModelParameter3);
            mCurrentBackgroundImgID ++;
            if(mCurrentBackgroundImgID == mBackgroundListLength) {
                mCurrentBackgroundImgID = 0;
                mCurrentTemplateImgID ++;
                if(mCurrentTemplateImgID == mTemplateListLength) break;
            }
        }
    }
    if(msg->data == "CreateLists") {
        mpPyCaller->getImageList(mTemplateFolder   + "/list.txt", mTemplateListLength,   mTemplateMinHeight,   mTemplateMinWidth,   mTemplateKeywords);
        mpPyCaller->getImageList(mBackgroundFolder + "/list.txt", mBackgroundListLength, mBackgroundMinHeight, mBackgroundMinWidth, mBackgroundKeywords);
    }
    if(msg->data == "CreateTemplateList") {
        mpPyCaller->getImageList(mTemplateFolder   + "/list.txt", mTemplateListLength,   mTemplateMinHeight,   mTemplateMinWidth,   mTemplateKeywords);
    }
    if(msg->data == "CreateBackgroundList") {
        mpPyCaller->getImageList(mBackgroundFolder + "/list.txt", mBackgroundListLength, mBackgroundMinHeight, mBackgroundMinWidth, mBackgroundKeywords);
    }
    if(msg->data == "DownloadTemplates") {
        mCurrentTemplateImgID = 0;
        while(downloadTemplateImage()) {
            mCurrentTemplateImgID ++;
        }
    }
    if(msg->data == "DownloadBackgrounds") {
        mCurrentBackgroundImgID = 0;
        while(downloadBackgroundImage()) {
            mCurrentBackgroundImgID++;
        }
    }
    if(msg->data == "DownloadImages") {
        mCurrentTemplateImgID = 0;
        while(downloadTemplateImage()) {
            mCurrentTemplateImgID ++;
        }
        mCurrentBackgroundImgID = 0;
        while(downloadBackgroundImage()) {
            mCurrentBackgroundImgID++;
        }
    }
    if(msg->data == "SetParameters") {
        setTemplateModelParameters();
    }
    if(msg->data == "EvaluateTemplates") {
        mCurrentTemplateImgID = 0;
        while(true) {
            if(mCurrentTemplateImgID == mTemplateListLength) break;
            std::string path =  mTemplateFolder + "/template_" + std::to_string(mCurrentTemplateImgID) + ".jpg";
            double acc_x, acc_y;
            mpTemplateEvaluation->evaluate(path, acc_x, acc_y);
            ROS_INFO_STREAM("ID: " << mCurrentTemplateImgID << " Acceptance: x: " << std::to_string(acc_x) << " y: " << std::to_string(acc_y));
            ++mCurrentTemplateImgID;
        }

    }
    if(msg->data == "ExportGroundTruth") {
        mpPyCaller->getGroundTruthData(mAnimation, mVideoOptions.frames,  "~/ground_truth");
    }
}

void AnimationRender::responseCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "InitOK") {
        std::string file = mPathVideos + "/" + mAnimation + "/" + mObject + "/video_" + std::to_string(mCurrentTemplateImgID) + "_" + std::to_string(mCurrentBackgroundImgID) + ".avi";
        if(mpVideoStream->openStream(file, mVideoOptions)) trackerControlPublish("ExportData");
        else initNextVideo();
    }
    if(msg->data == "ExportDataOK") {
        initNextVideo();
    }
}

void AnimationRender::getTestParameters()
{
    mpNodeHandle->param<std::string>("video_folder",      mPathVideos,       ros::package::getPath("animation_render") + "/videos");
    mpNodeHandle->param<std::string>("template_folder",   mTemplateFolder,   ros::package::getPath("animation_render") + "/templates");
    mpNodeHandle->param<std::string>("background_folder", mBackgroundFolder, ros::package::getPath("animation_render") + "/backgrounds");

    mpNodeHandle->param<int>("background_list_length", mBackgroundListLength, 5);
    mpNodeHandle->param<int>("template_list_length",   mTemplateListLength, 5);

    mpNodeHandle->param<bool>("white_background",   mUseWhiteBackground, false);

    // Video parameters
    mpNodeHandle->param<int>("frames",              mVideoOptions.frames, 100);
    mpNodeHandle->param<int>("fps",                 mVideoOptions.fps,    25);
    mpNodeHandle->param<int>("render_image_width",  mVideoOptions.width,  600);
    mpNodeHandle->param<int>("render_image_height", mVideoOptions.height, 600);
    mpNodeHandle->param<double>("focal_length",     mVideoOptions.focal_length, 35.0);
    mpNodeHandle->param<double>("sensor_width",     mVideoOptions.sensor_width, 32.0);

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
    mpNodeHandle->param<double>("threshold_acceptance", mpTemplateEvaluation->mAcceptanceThreshold, 0);
    mpNodeHandle->param<double>("threshold_gradient"  , mpTemplateEvaluation->mGradThreshold,       0);

    mpNodeHandle->param<double>("init_R_range", mInitTranslationRange, 0);
    mpNodeHandle->param<double>("init_T_range", mInitRotationRange, 0);
}

void AnimationRender::setTemplateModelParameters()
{
    //mpPyCaller->getInitPose(mAnimation, mInitPose);
    ROS_INFO_STREAM("Setting template "  << std::to_string(mCurrentTemplateImgID) << " and model parameters.");

    mTemplateParameters[0]["filename"] = mTemplateFolder + "/template_" + std::to_string(mCurrentTemplateImgID) + ".jpg";
    mTemplateParameters[0]["resize"]   = mResize;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> disR(-mInitRotationRange, mInitRotationRange);
    std::uniform_real_distribution<> disT(-mInitTranslationRange, mInitTranslationRange);

    mModelParameters[0]["model_name"] = mObject;
    mModelParameters[0]["x"]          = mInitPose.x  + disT(gen);
    mModelParameters[0]["y"]          = mInitPose.y  + disT(gen);
    mModelParameters[0]["z"]          = mInitPose.z  + disT(gen);
    mModelParameters[0]["ax"]         = mInitPose.ax + disR(gen);
    mModelParameters[0]["ay"]         = mInitPose.ay + disR(gen);
    mModelParameters[0]["az"]         = mInitPose.az + disR(gen);
    mModelParameters[0]["length"]     = mModelParameter1;
    mModelParameters[0]["radius"]     = mModelParameter2;
    mModelParameters[0]["height"]     = mModelParameter1;
    mModelParameters[0]["width"]      = mModelParameter2;

    // set new parameters
    mGlobalNodeHandle.setParam("/mdltracker/templates", mTemplateParameters);
    mGlobalNodeHandle.setParam("/mdltracker/models",    mModelParameters);
}

void AnimationRender::trackerControlPublish(std::string msg)
{
    std_msgs::String toSend;
    toSend.data = msg;
    mPubTrackerControl.publish(toSend);
}
