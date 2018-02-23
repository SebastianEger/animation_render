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
    nH_(nH)
{
    _nH_ = ros::NodeHandle();
    pyCaller = new PythonCaller(&_nH_);
    vstream = new VideoStream(_nH_);
    vstream->fps = 25;

    pubTrackerControl_ = _nH_.advertise<std_msgs::String>("/tplsearch/control", 1);
    pubRenderControl_ = _nH_.advertise<std_msgs::String>("/render/control", 1000);
    subControl_ = _nH_.subscribe<std_msgs::String>("/tplsearch_test/control", 1, &TplTester::controlCallback, this);

    currentTemplateImgID_ = 0;
    currentBackgroundImgID_ = 0;

    getTestParameters();

    if(!skipGroundTruth_) {
        std::string dataPath;
        _nH_.param<std::string>("/tplsearch/data_path", dataPath, "~/tplsearch_data");
        pyCaller->get_ground_truth_data(animation_, frames_, dataPath + "/ground_truth");
    }

    templates = XmlRpc::XmlRpcValue();
    templates[0] = XmlRpc::XmlRpcValue();
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
        setTemplateSearchParameters();
        trackerControlPublish("Init");
    }
}

void TplTester::getTestParameters()
{
    nH_->param<bool>("autostart", autostart_, false);
    nH_->param<int>("list_length", listLength_, 100);

    //
    nH_->param<bool>("skip_render",         skipRender_,        false);
    nH_->param<bool>("skip_img_list",       skipImgList_,       false);
    nH_->param<bool>("skip_ground_truth",   skipGroundTruth_,   false);
    nH_->param<bool>("skip_image_download", skipImageDownload_, false);

    // Video parameters
    nH_->param<int>("frames",              frames_, 100);
    nH_->param<int>("fps",                 fps_,    25);
    nH_->param<int>("render_image_width",  res_x_,  600);
    nH_->param<int>("render_image_height", res_y_,  600);

    nH_->param<std::string>("animation",           animation_,           "Rotation");
    nH_->param<std::string>("object",              object_,              "Cylinder");
    nH_->param<std::string>("template_keywords",   template_keywords_,   "bird");
    nH_->param<std::string>("background_keywords", background_keywords_, "plane");

    // Model parameters
    nH_->param<std::string>("model_parameter_1", mp1, "0.05");
    nH_->param<std::string>("model_parameter_2", mp2, "0.01");
    nH_->param<std::string>("model_parameter_3", mp3, "200");

    nH_->param<int>("template_min_height",   template_min_height_,   500);
    nH_->param<int>("template_min_width",    template_min_width_,    500);
    nH_->param<int>("background_min_height", background_min_height_, 500);
    nH_->param<int>("background_min_width",  background_min_width_,  500);
}

void TplTester::getTplsearchParameters()
{

}

void TplTester::setTemplateSearchParameters()
{
    ROS_INFO("Setting template parameters.");

    for(int i = 0; i < templates.size(); i++) {
        templates[i]["filename"] = ros::package::getPath("tplsearch_test") + "/img/template_image.jpg";
        templates[i]["x"] = x_;
        templates[i]["y"] = y_;
        templates[i]["z"] = z_;
        templates[i]["ax"] = ax_;
        templates[i]["ay"] = ay_;
        templates[i]["az"] = az_;
    }

    // set new parameters
    _nH_.setParam("/tplsearch/templates", templates);
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
