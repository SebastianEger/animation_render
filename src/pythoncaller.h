#ifndef PYTHONCALLER_H
#define PYTHONCALLER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <stdio.h>

struct Pose {
    double x;
    double y;
    double z;
    double ax;
    double ay;
    double az;
};

class PythonCaller
{
public:
    PythonCaller(ros::NodeHandle *nh);

    /*!
     * \brief renderVideo Render video
     * \param frames Number frames
     * \param fps Frames per second
     * \param object Name of object: Plane, Cylinder
     * \param animation Name of animation
     * \param res_x Resolution x
     * \param res_y Resolution y
     * \param mp1 Model parameter 1
     * \param mp2 Model parameter 2
     * \param mp3 Model parameter 3
     */
    void renderVideo(int frames, int fps, std::string object, std::string animation, int res_x, int res_y, double mp1, double mp2, double mp3);

    /*!
     * \brief getTemplateImageList Creates a list of template image urls
     * \param length Length of list
     * \param min_height Minimal height of images
     * \param min_width Minimal width of images
     * \param keywords Some keywords to search for
     */
    void getTemplateImageList(int length, int min_height, int min_width, std::string keywords);

    /*!
     * \brief getBackgroundImageList Creates a list of background image urls
     * \param length Length of list
     * \param min_height Minimal height of images
     * \param min_width Minimal width of images
     * \param keywords Some keywords to search for
     */
    void getBackgroundImageList(int length, int min_height, int min_width, std::string keywords);

    /*!
     * \brief downloadTemplateImage Download template image from list
     * \param nr Index of image
     */
    void downloadTemplateImage(int nr);

    /*!
     * \brief downloadBackgroundImage Donwload background image from list
     * \param nr Index of image
     */
    void downloadBackgroundImage(int nr);

    /*!
     * \brief getGroundTruthData Get the ground truth data of given animation
     * \param animation Name of animation
     * \param frames Number of frames
     * \param path output path
     */
    void getGroundTruthData(std::string animation, int frames, std::string path);

    /*!
     * \brief getInitPose Get init pose of animation
     * \param animation Name of animation
     * \param x x position
     * \param y y position
     * \param z z position
     * \param ax angle x
     * \param ay angle y
     * \param az angle z
     */
    void getInitPose(std::string animation, Pose &initPose);

private:

    /*!
     * \brief mpNodeHandle Pointer to ros node handle
     */
    ros::NodeHandle *mpNodeHandle;

    /*!
     * \brief mPkgPath Path to this node
     */
    std::string mPkgPath;

    /*!
     * \brief mPython3 python3 command
     */
    std::string mPython3;

    /*!
     * \brief mPython python command
     */
    std::string mPython;

    /*!
     * \brief execute_script Executes python scripts
     * \param script name of script
     * \param version python versionn, 2 or 3
     * \return Output of executed python script
     */
    FILE* execute_script(std::string script, int version);

    /*!
     * \brief execute_script Executes python script
     * \param script name of script
     */
    void execute_script(std::string script);
};

#endif // PYTHONCALLER_H
