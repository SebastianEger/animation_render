#ifndef PYTHONCALLER_H
#define PYTHONCALLER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <stdio.h>

class PythonCaller
{
public:
    PythonCaller(ros::NodeHandle *nh);

    void render_video(int frames, int fps, std::string object, std::string animation, int res_x, int res_y, double mp1, double mp2, double mp3);

    void get_template_image_list(int length, int min_height, int min_width, std::string keywords);
    void get_background_image_list(int length, int min_height, int min_width, std::string keywords);

    void download_template_image(int nr);
    void download_background_image(int nr);

    void get_ground_truth_data(std::string animation, int frames, std::string path);

    void get_init_pose(std::string animation, double &x, double &y, double &z, double &ax, double &ay, double &az);

private:

    ros::NodeHandle *nH_;

    std::string pkg_path_;
    std::string python3_, python_;

    FILE* execute_script(std::string script, int version);
    void execute_script(std::string script);
};

#endif // PYTHONCALLER_H
