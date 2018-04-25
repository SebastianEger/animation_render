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
     * \brief openStream Reads video file and streams it over camera topic
     * \param path Path of video file
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
    /*!
     * \brief mNodeHandle
     */
    ros::NodeHandle mNodeHandle;

    /*!
     * \brief mpVideoCapture Point to VideoCapture class
     */
    cv::VideoCapture *mpVideoCapture;

    /*!
     * \brief mFrameID Frame ID for header
     */
    std::string mFrameID;

    /*!
     * \brief mCameraName Name of camera
     */
    std::string mCameraName;

    /*!
     * \brief getDefaultCameraInfoFromImage
     * \param img
     * \return
     */
    sensor_msgs::CameraInfo getDefaultCameraInfoFromImage(sensor_msgs::ImagePtr img);

    /*!
     * \brief mpImageTransport
     */
    image_transport::ImageTransport *mpImageTransport;

    /*!
     * \brief mPub Image publisher
     */
    image_transport::CameraPublisher mPub;
};

#endif // VIDEOSTREAM_H
