#ifndef PYTHONCALLER_H
#define PYTHONCALLER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <stdio.h>

class PythonCaller
{
public:
    PythonCaller(ros::NodeHandle *nh);

    /*!
     * \brief render_video Render video
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
    void render_video(int frames, int fps, std::string object, std::string animation, int res_x, int res_y, double mp1, double mp2, double mp3);

    /*!
     * \brief get_template_image_list Creates a list of template image urls
     * \param length Length of list
     * \param min_height Minimal height of images
     * \param min_width Minimal width of images
     * \param keywords Some keywords to search for
     */
    void get_template_image_list(int length, int min_height, int min_width, std::string keywords);

    /*!
     * \brief get_background_image_list Creates a list of background image urls
     * \param length Length of list
     * \param min_height Minimal height of images
     * \param min_width Minimal width of images
     * \param keywords Some keywords to search for
     */
    void get_background_image_list(int length, int min_height, int min_width, std::string keywords);

    /*!
     * \brief download_template_image Download template image from list
     * \param nr Index of image
     */
    void download_template_image(int nr);

    /*!
     * \brief download_background_image Donwload background image from list
     * \param nr Index of image
     */
    void download_background_image(int nr);

    /*!
     * \brief get_ground_truth_data Get the ground truth data of given animation
     * \param animation Name of animation
     * \param frames Number of frames
     * \param path output path
     */
    void get_ground_truth_data(std::string animation, int frames, std::string path);

    /*!
     * \brief get_init_pose Get init pose of animation
     * \param animation Name of animation
     * \param x x position
     * \param y y position
     * \param z z position
     * \param ax angle x
     * \param ay angle y
     * \param az angle z
     */
    void get_init_pose(std::string animation, double &x, double &y, double &z, double &ax, double &ay, double &az);

private:

    ros::NodeHandle *mpNodeHandle;

    std::string pkg_path_;
    std::string python3_, python_;

    /*!
     * \brief execute_script
     * \param script
     * \param version
     * \return
     */
    FILE* execute_script(std::string script, int version);

    /*!
     * \brief execute_script
     * \param script
     */
    void execute_script(std::string script);
};

#endif // PYTHONCALLER_H
