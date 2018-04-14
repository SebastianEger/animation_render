#ifndef VIDEOSTREAM_H
#define VIDEOSTREAM_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <boost/assign/list_of.hpp>


class VideoStream
{
public:
    VideoStream(ros::NodeHandle rosH);

    /*!
     * \brief openStream
     * \param path
     */
    void openStream(std::string path);

    /*!
     * \brief mWidth
     */
    int mWidth;

    /*!
     * \brief mHeight
     */
    int mHeight;

    /*!
     * \brief mFPS
     */
    int mFPS;

private:
    ros::NodeHandle rosNodeHandle_;

    cv::VideoCapture *cap;

    std::string frameID_;
    std::string cameraName_;

    sensor_msgs::CameraInfo getDefaultCameraInfoFromImage(sensor_msgs::ImagePtr img);

    image_transport::ImageTransport *it;
    image_transport::CameraPublisher pub;
};

#endif // VIDEOSTREAM_H
