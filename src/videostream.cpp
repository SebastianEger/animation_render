#include "videostream.h"

VideoStream::VideoStream(ros::NodeHandle rosH) :
    rosNodeHandle_(rosH)
{
    cap = new cv::VideoCapture();
    it  = new image_transport::ImageTransport(rosH);

    pub = it->advertiseCamera("/camera", 1);

    frameID_ = "camera";
}

void VideoStream::openStream(std::string path)
{
    cap->open(path);

    if(!cap->isOpened()){
        ROS_ERROR_STREAM("Could not open the stream.");
        return;
    }

    // set width and height
    if (mWidth != 0 && mHeight != 0){
        cap->set(CV_CAP_PROP_FRAME_WIDTH, mWidth);
        cap->set(CV_CAP_PROP_FRAME_HEIGHT, mHeight);
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo cam_info_msg;
    std_msgs::Header header;
    header.frame_id = frameID_;

    camera_info_manager::CameraInfoManager cam_info_manager(rosNodeHandle_, cameraName_, "");

    cam_info_msg = cam_info_manager.getCameraInfo();

    ros::Rate r(mFPS);
    while (cap->get(CV_CAP_PROP_POS_FRAMES) != cap->get(CV_CAP_PROP_FRAME_COUNT)) {
        *cap >> frame;

        if (pub.getNumSubscribers() > 0){

            if(!frame.empty()) {
                msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

                if (cam_info_msg.distortion_model == ""){
                    ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
                    cam_info_msg = getDefaultCameraInfoFromImage(msg);
                    cam_info_manager.setCameraInfo(cam_info_msg);
                }
                pub.publish(*msg, cam_info_msg, ros::Time::now());
            } else {
                ROS_ERROR_STREAM("Empty frame.");
                return;
            }

            ros::spinOnce();
        }
        r.sleep();
    }
}

sensor_msgs::CameraInfo VideoStream::getDefaultCameraInfoFromImage(sensor_msgs::ImagePtr img)
{
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = img->header.frame_id;
    // Fill image size
    cam_info_msg.height = img->height;
    cam_info_msg.width = img->width;
    ROS_INFO_STREAM("The image width is: " << img->width);
    ROS_INFO_STREAM("The image height is: " << img->height);
    // Add the most common distortion model as sensor_msgs/CameraInfo says
    cam_info_msg.distortion_model = "plumb_bob";
    // Don't let distorsion matrix be empty
    cam_info_msg.D.resize(5, 0.0);
    // Give a reasonable default intrinsic camera matrix
    cam_info_msg.K = boost::assign::list_of(1.0) (0.0) (img->width/2.0)
                                           (0.0) (1.0) (img->height/2.0)
                                           (0.0) (0.0) (1.0);
    // Give a reasonable default rectification matrix
    cam_info_msg.R = boost::assign::list_of (1.0) (0.0) (0.0)
                                            (0.0) (1.0) (0.0)
                                            (0.0) (0.0) (1.0);
    // Give a reasonable default projection matrix
    cam_info_msg.P = boost::assign::list_of (1.0) (0.0) (img->width/2.0) (0.0)
                                            (0.0) (1.0) (img->height/2.0) (0.0)
                                            (0.0) (0.0) (1.0) (0.0);
    return cam_info_msg;
}
