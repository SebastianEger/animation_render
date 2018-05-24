#include "videostream.h"

VideoStream::VideoStream(ros::NodeHandle rosH) :
    mNodeHandle(rosH)
{
    mpVideoCapture    = new cv::VideoCapture();
    mpImageTransport  = new image_transport::ImageTransport(rosH);

    // Init publisher
    mPub = mpImageTransport->advertiseCamera("/camera", 1);

    mFrameID = "camera";
}

VideoStream::~VideoStream()
{
    delete mpVideoCapture;
    delete mpImageTransport;
}

bool VideoStream::openStream(std::string path, VideoOptions options)
{
    mpVideoCapture->open(path);

    if( !mpVideoCapture->isOpened() ){
        ROS_ERROR_STREAM("Could not open the stream.");
        return false;
    }

    // set width and height
    if (options.width != 0 && options.height != 0){
        mpVideoCapture->set(CV_CAP_PROP_FRAME_WIDTH, options.width);
        mpVideoCapture->set(CV_CAP_PROP_FRAME_HEIGHT, options.height);
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo cam_info_msg;
    std_msgs::Header header;
    header.frame_id = mFrameID;

    camera_info_manager::CameraInfoManager cam_info_manager(mNodeHandle, mCameraName, "");

    cam_info_msg.height = options.height;
    cam_info_msg.width  = options.width;
    cam_info_msg.distortion_model = "plumb_bob";
    cam_info_msg.D.resize(5, 0.0);
    cam_info_msg.K = boost::assign::list_of( options.focal_length / options.sensor_width * double(options.width) ) (0.0) (double(options.width)/2.0)
                                           ( options.focal_length / options.sensor_width * double(options.height) ) (1.0) (double(options.height)/2.0)
                                           (0.0) (0.0) (1.0);
    cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
                                            (0.0) (1.0) (0.0)
                                            (0.0) (0.0) (1.0);
    cam_info_msg.P = boost::assign::list_of (1.0) (0.0) (double(options.width)/2.0) (0.0)
                                            (0.0) (1.0) (double(options.height)/2.0) (0.0)
                                            (0.0) (0.0) (1.0) (0.0);

    // init rate
    ros::Rate r(options.fps);

    while (mpVideoCapture->get(CV_CAP_PROP_POS_FRAMES) != mpVideoCapture->get(CV_CAP_PROP_FRAME_COUNT)) {
        // get frame
        *mpVideoCapture >> frame;
        // only publish next frame if we have a subscriber
        if (mPub.getNumSubscribers() > 0) {
            if(!frame.empty()) {
                // convert frame
                msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                cam_info_msg.header.frame_id = msg->header.frame_id;
                cam_info_manager.setCameraInfo(cam_info_msg);
                mPub.publish(*msg, cam_info_msg, ros::Time::now());
            } else {
                ROS_ERROR_STREAM("Empty frame.");
                return false;
            }
            ros::spinOnce();
        }
        r.sleep();
    }
    return true;
}

