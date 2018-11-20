#include "pythoncaller.h"

PythonCaller::PythonCaller(ros::NodeHandle *nh) :
    mpNodeHandle(nh)
{
    mPkgPath = ros::package::getPath("animation_render");
    mPython3 = "python3 ";
    mPython  = "python ";
}

void PythonCaller::renderVideo(std::string video_name, std::string template_name, std::string background_name,
                               VideoOptions video_options, std::string object, std::string animation, double mp1, double mp2, double mp3)
{
    std::string py_file        = "render_video.py";
    std::string opt_frames     = " --frames " + std::to_string(video_options.frames);
    std::string opt_fps        = " --fps "    + std::to_string(video_options.fps);
    std::string opt_object     = " --obj "    + object;
    std::string opt_animation  = " --anim "   + animation;
    std::string opt_resolution = " --res "    + std::to_string(video_options.width) + " " + std::to_string(video_options.height);
    std::string opt_size       = " --mp "     + std::to_string(mp1) + " " +  std::to_string(mp2) + " " +  std::to_string(mp3);
    std::string opt_sensor     = " --sensor " + std::to_string(video_options.sensor_width);
    std::string opt_focal      = " --focal "  + std::to_string(video_options.focal_length);

    // DO NOT CHANGE
    std::string tpl_img = " --tpl_img " + template_name + ".jpg";
    std::string bgr_img = " --bgr_img " + background_name + ".jpg";
    std::string output  = " --out " + video_name + ".avi";

    execute_script(py_file
                   + opt_frames
                   + opt_fps
                   + opt_object
                   + opt_animation
                   + opt_resolution
                   + opt_size
                   + opt_focal
                   + opt_sensor
                   + tpl_img
                   + bgr_img
                   + output);
}

void PythonCaller::getImageList(std::string filename, int length, int min_height, int min_width, std::string keywords)
{
    std::string py_file      = "generate_image_list.py";
    std::string opt_length   = " --length "   + std::to_string(length);
    std::string opt_min_size = " --size "     + std::to_string(min_width) + " " + std::to_string(min_height);
    std::string opt_path     = " --path "     + filename;
    std::string opt_keywords = " --keywords " + keywords;

    execute_script(py_file + opt_length + opt_min_size + opt_path + opt_keywords);
}

void PythonCaller::downloadImage(std::string filename, std::string list, int nr)
{
    std::string py_file = "download_image.py";

    std::string opt_path   = " --path " + filename + ".jpg";
    std::string opt_list   = " --img_list " + list;
    std::string opt_number = " --image_nr " + std::to_string(nr);

    execute_script(py_file + opt_path + opt_list + opt_number);
}



void PythonCaller::getGroundTruthData(std::string animation, int frames, std::string path)
{
     std::string py_file = "get_ground_truth.py";

     std::string opt_path = " --path " + path;
     std::string opt_anim = " --anim " + animation;
     std::string opt_frames = " --frames " + std::to_string(frames);

     execute_script(py_file + opt_path + opt_anim + opt_frames);
}

void PythonCaller::getInitPose(std::string animation, Pose &initPose)
{
    std::string py_file = "get_init_pose.py";
    std::string opt_anim = " --anim " + animation;

    FILE* out = execute_script(py_file + opt_anim, 3);
    fscanf(out, "%lf %lf %lf %lf %lf %lf", &initPose.x, &initPose.y, &initPose.z, &initPose.ax, &initPose.ay, &initPose.az);

    delete out;
}

FILE *PythonCaller::execute_script(std::string script, int version)
{
    ROS_INFO_STREAM("Executing " << script);
    std::string execute_string;
    if(version == 2) execute_string = mPython + mPkgPath + "/scripts/" + script;
    if(version == 3) execute_string = mPython3 + mPkgPath + "/scripts/" + script;
    return( popen( execute_string.c_str(), "r") );
}

void PythonCaller::execute_script(std::string script)
{
    ROS_INFO_STREAM("Executing " << script);
    std::string execute_string = mPython3 + mPkgPath + "/scripts/" + script;
    system(execute_string.c_str());
}
