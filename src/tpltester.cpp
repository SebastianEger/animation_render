#include "tpltester.h"


int main( int argc, char** argv )
{
    ros::init(argc, argv, "tpltester");
    ros::NodeHandle nH("~");

    TplTester tester(&nH);
    if(tester.autostart_) tester.start();

    printf("tplsearch: Entering ros::spin()\n");
    ros::spin();

    return 0;

}

TplTester::TplTester(ros::NodeHandle *nH) :
    mpNodeHandle(nH)
{
    mGlobalNodeHandle = ros::NodeHandle();
    pyCaller = new PythonCaller(&mGlobalNodeHandle);
    vstream = new VideoStream(mGlobalNodeHandle);
    vstream->fps = 25;

    pubTrackerControl_ = mGlobalNodeHandle.advertise<std_msgs::String>("/tplsearch/control", 1);
    pubRenderControl_ = mGlobalNodeHandle.advertise<std_msgs::String>("/render/control", 1000);
    subControl_ = mGlobalNodeHandle.subscribe<std_msgs::String>("/tplsearch_test/control", 1, &TplTester::controlCallback, this);

    currentTemplateImgID_ = 0;
    currentBackgroundImgID_ = 0;

    getTestParameters();

    if(!skipGroundTruth_) {
        std::string dataPath;
        mGlobalNodeHandle.param<std::string>("/tplsearch/data_path", dataPath, "~/tplsearch_data");
        pyCaller->get_ground_truth_data(animation_, frames_, dataPath + "/ground_truth");
    }

    mTemplateParameters = XmlRpc::XmlRpcValue();
    mTemplateParameters[0] = XmlRpc::XmlRpcValue();

    mModelParameters = XmlRpc::XmlRpcValue();
    mModelParameters[0] = XmlRpc::XmlRpcValue();
}

void TplTester::start()
{
    pyCaller->get_init_pose(animation_, x_, y_, z_, ax_, ay_, az_);
    if(!skipImgList_) {
        pyCaller->get_template_image_list(listLength_, template_min_height_, template_min_width_, template_keywords_);
        pyCaller->get_background_image_list(listLength_, background_min_height_, background_min_width_, background_keywords_);
    }
    setTemplateSearchParameters();
    trackerControlPublish("Init");
}

void TplTester::controlCallback(const std_msgs::String::ConstPtr &msg)
{
    // TPLSEARCH_TEST COMMANDS
    if(msg->data == "StartTest") {
        start();
    }
    if(msg->data == "RenderVideo") {
        pyCaller->render_video(frames_, fps_, object_, animation_, res_x_, res_y_, mp1, mp2, mp3);
    }
    if(msg->data == "CreateImageList") {
        pyCaller->get_template_image_list(listLength_, template_min_height_, template_min_width_, template_keywords_);
        pyCaller->get_background_image_list(listLength_, background_min_height_, background_min_width_, background_keywords_);
    }
    if(msg->data == "DownloadNextImage") {
        pyCaller->download_template_image(currentTemplateImgID_);
        pyCaller->download_background_image(currentBackgroundImgID_);

        currentBackgroundImgID_ ++;
        if(currentBackgroundImgID_ == listLength_) {
            currentTemplateImgID_ ++;
            currentBackgroundImgID_ = 0;
        }
    }
    if(msg->data == "RunVideo") {
        vstream->fps = fps_;
        vstream->openStream(ros::package::getPath("tplsearch_test") + "/render/video.avi");
        trackerControlPublish("ExportData");
    }
    if(msg->data == "SetParameters") {
        setTemplateSearchParameters();
    }

    // TPLSEARCH RESPONSES
    if(msg->data == "InitOK") {
        if(!skipRender_) pyCaller->render_video(frames_, fps_, object_, animation_, res_x_, res_y_, mp1, mp2, mp3);
        vstream->fps = fps_;
        vstream->openStream(ros::package::getPath("tplsearch_test") + "/render/video.avi");
        trackerControlPublish("ExportData");
    }
    if(msg->data == "ExportDataOK") {
        currentBackgroundImgID_ ++;
        if(currentBackgroundImgID_ == listLength_) {
            currentTemplateImgID_ ++;
            currentBackgroundImgID_ = 0;
        }
        if(currentTemplateImgID_ == listLength_) {
            trackerControlPublish("Shutdown");
            ros::shutdown();
            return;
        }
        if(!skipImageDownload_) {
            pyCaller->download_template_image(currentTemplateImgID_);
            pyCaller->download_background_image(currentBackgroundImgID_);
        }


        trackerControlPublish("Init");
    }
}

void TplTester::getTestParameters()
{
    mpNodeHandle->param<bool>("autostart", autostart_, false);
    mpNodeHandle->param<int>("list_length", listLength_, 100);

    //
    mpNodeHandle->param<bool>("skip_render",         skipRender_,        false);
    mpNodeHandle->param<bool>("skip_img_list",       skipImgList_,       false);
    mpNodeHandle->param<bool>("skip_ground_truth",   skipGroundTruth_,   false);
    mpNodeHandle->param<bool>("skip_image_download", skipImageDownload_, false);

    // Video parameters
    mpNodeHandle->param<int>("frames",              frames_, 100);
    mpNodeHandle->param<int>("fps",                 fps_,    25);
    mpNodeHandle->param<int>("render_image_width",  res_x_,  600);
    mpNodeHandle->param<int>("render_image_height", res_y_,  600);

    mpNodeHandle->param<std::string>("animation",           animation_,           "Rotation");
    mpNodeHandle->param<std::string>("object",              object_,              "Cylinder");
    mpNodeHandle->param<std::string>("template_keywords",   template_keywords_,   "bird");
    mpNodeHandle->param<std::string>("background_keywords", background_keywords_, "plane");

    // Model parameters
    mpNodeHandle->param<double>("model_parameter_1", mp1, 0.05);
    mpNodeHandle->param<double>("model_parameter_2", mp2, 0.01);
    mpNodeHandle->param<double>("model_parameter_3", mp3, 200);

    std::cout << mp1 << mp2 << mp3 << std::endl;

    mpNodeHandle->param<int>("template_min_height",   template_min_height_,   500);
    mpNodeHandle->param<int>("template_min_width",    template_min_width_,    500);
    mpNodeHandle->param<int>("background_min_height", background_min_height_, 500);
    mpNodeHandle->param<int>("background_min_width",  background_min_width_,  500);
}

void TplTester::getTplsearchParameters()
{

}

void TplTester::setTemplateSearchParameters()
{
    ROS_INFO("Setting template parameters.");

    for(int i = 0; i < mTemplateParameters.size(); i++) {
        mTemplateParameters[i]["filename"] = ros::package::getPath("tplsearch_test") + "/img/template_image.jpg";
        mTemplateParameters[i]["resize"]   = 200;
    }

    for(int i = 0; i < mModelParameters.size(); i++) {
        mModelParameters[i]["model_name"] = "Point" + object_;
        mModelParameters[i]["x"]      = x_;
        mModelParameters[i]["y"]      = y_;
        mModelParameters[i]["z"]      = z_;
        mModelParameters[i]["ax"]     = ax_;
        mModelParameters[i]["ay"]     = ay_;
        mModelParameters[i]["az"]     = az_;
        mModelParameters[i]["length"] = mp1;
        mModelParameters[i]["radius"] = mp2;
        mModelParameters[i]["width"]  = mp1;
        mModelParameters[i]["height"] = mp2;
    }
    // set new parameters
    mGlobalNodeHandle.setParam("/tplsearch/templates", mTemplateParameters);
    mGlobalNodeHandle.setParam("/tplsearch/models",    mModelParameters);
}

void TplTester::renderControlPublish(std::string msg)
{
    std_msgs::String toSend;
    toSend.data = msg;
    pubRenderControl_.publish(toSend);
}

void TplTester::trackerControlPublish(std::string msg)
{
    std_msgs::String toSend;
    toSend.data = msg;
    pubTrackerControl_.publish(toSend);
}
