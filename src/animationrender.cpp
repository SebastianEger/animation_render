#include "animationrender.h"


int main( int argc, char** argv )
{
    ros::init(argc, argv, "animation_render");
    ros::NodeHandle nH("~");

    AnimationRender tester(&nH);
    if(tester.mAutostart) tester.start();

    ros::spin();

    return 0;

}

AnimationRender::AnimationRender(ros::NodeHandle *nH) :
    mpNodeHandle(nH)
{
    mGlobalNodeHandle = ros::NodeHandle();

    mPyCaller =     new PythonCaller(&mGlobalNodeHandle);
    mVideoStream =  new VideoStream(mGlobalNodeHandle);

    // Set fps to default 25
    mVideoStream->mFPS = 25;

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

    if(!skipGroundTruth_) {
        std::string dataPath;
        mGlobalNodeHandle.param<std::string>("/tplsearch/data_path", dataPath, "~/tplsearch_data");
        mPyCaller->get_ground_truth_data(animation_, mFrames, dataPath + "/ground_truth");
    }

    // Init template parameters
    mTemplateParameters    = XmlRpc::XmlRpcValue();
    mTemplateParameters[0] = XmlRpc::XmlRpcValue();

    // Init model parameters
    mModelParameters    = XmlRpc::XmlRpcValue();
    mModelParameters[0] = XmlRpc::XmlRpcValue();
}

void AnimationRender::start()
{
    mPyCaller->get_init_pose(animation_, x_, y_, z_, ax_, ay_, az_);
    if(!skipImgList_) {
        mPyCaller->get_template_image_list(mListLength, template_min_height_, template_min_width_, template_keywords_);
        mPyCaller->get_background_image_list(mListLength, background_min_height_, background_min_width_, background_keywords_);
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
        mPyCaller->render_video(mFrames, mFPS, object_, animation_, res_x_, res_y_, mModelParameter1, mModelParameter2, mModelParameter3);
    }
    if(msg->data == "CreateImageList") {
        mPyCaller->get_template_image_list(mListLength, template_min_height_, template_min_width_, template_keywords_);
        mPyCaller->get_background_image_list(mListLength, background_min_height_, background_min_width_, background_keywords_);
    }
    if(msg->data == "DownloadNextImage") {
        mPyCaller->download_template_image(mCurrentTemplateImgID);
        mPyCaller->download_background_image(mCurrentBackgroundImgID);

        mCurrentBackgroundImgID ++;
        if(mCurrentBackgroundImgID == mListLength) {
            mCurrentTemplateImgID ++;
            mCurrentBackgroundImgID = 0;
        }
    }
    if(msg->data == "RunVideo") {
        mVideoStream->mFPS = mFPS;
        mVideoStream->openStream(ros::package::getPath("animation_render") + "/render/video.avi");
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
        mTemplateEvaluation.evaluateTemplate(path);
    }
}

void AnimationRender::responseCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "InitOK") {
        if(!skipRender_) mPyCaller->render_video(mFrames, mFPS, object_, animation_, res_x_, res_y_, mModelParameter1, mModelParameter2, mModelParameter3);
        mVideoStream->mFPS = mFPS;
        mVideoStream->openStream(ros::package::getPath("animation_render") + "/render/video.avi");
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
        if(!skipImageDownload_) {
            mPyCaller->download_template_image(mCurrentTemplateImgID);
            mPyCaller->download_background_image(mCurrentBackgroundImgID);
        }


        trackerControlPublish("Init");
    }
}

void AnimationRender::getTestParameters()
{
    mpNodeHandle->param<bool>("autostart", mAutostart, false);
    mpNodeHandle->param<int>("list_length", mListLength, 100);

    //
    mpNodeHandle->param<bool>("skip_render",         skipRender_,        false);
    mpNodeHandle->param<bool>("skip_img_list",       skipImgList_,       false);
    mpNodeHandle->param<bool>("skip_ground_truth",   skipGroundTruth_,   false);
    mpNodeHandle->param<bool>("skip_image_download", skipImageDownload_, false);

    // Video parameters
    mpNodeHandle->param<int>("frames",              mFrames, 100);
    mpNodeHandle->param<int>("fps",                 mFPS,    25);
    mpNodeHandle->param<int>("render_image_width",  res_x_,  600);
    mpNodeHandle->param<int>("render_image_height", res_y_,  600);

    mpNodeHandle->param<std::string>("animation",           animation_,           "Rotation");
    mpNodeHandle->param<std::string>("object",              object_,              "Cylinder");
    mpNodeHandle->param<std::string>("template_keywords",   template_keywords_,   "bird");
    mpNodeHandle->param<std::string>("background_keywords", background_keywords_, "plane");

    // Model parameters
    mpNodeHandle->param<double>("model_parameter_1", mModelParameter1, 0.05);
    mpNodeHandle->param<double>("model_parameter_2", mModelParameter2, 0.01);
    mpNodeHandle->param<double>("model_parameter_3", mModelParameter3, 200);

    mpNodeHandle->param<int>("template_min_height",   template_min_height_,   500);
    mpNodeHandle->param<int>("template_min_width",    template_min_width_,    500);
    mpNodeHandle->param<int>("background_min_height", background_min_height_, 500);
    mpNodeHandle->param<int>("background_min_width",  background_min_width_,  500);

    // Template parameters
    mpNodeHandle->param<int>("template_resize",  mResize,  100);
}

void AnimationRender::setTemplateModelParameters()
{
    ROS_INFO("Setting template and model parameters.");

    mTemplateParameters[0]["filename"] = ros::package::getPath("animation_render") + "/img/template_image.jpg";
    mTemplateParameters[0]["resize"]   = mResize;

    mModelParameters[0]["model_name"] = object_;
    mModelParameters[0]["x"]      = x_;
    mModelParameters[0]["y"]      = y_;
    mModelParameters[0]["z"]      = z_;
    mModelParameters[0]["ax"]     = ax_;
    mModelParameters[0]["ay"]     = ay_;
    mModelParameters[0]["az"]     = az_;
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
