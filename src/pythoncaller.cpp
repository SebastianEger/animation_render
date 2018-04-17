#include "pythoncaller.h"

PythonCaller::PythonCaller(ros::NodeHandle *nh) :
    mpNodeHandle(nh)
{
    mPkgPath = ros::package::getPath("animation_render");
    mPython3 = "python3 ";
    mPython = "python ";
}

void PythonCaller::render_video(int frames, int fps, std::string object, std::string animation, int res_x, int res_y, double mp1, double mp2, double mp3)
{
    std::string py_file = "render_video.py";
    std::string opt_frames = " --frames " + std::to_string(frames);
    std::string opt_fps = " --fps " + std::to_string(fps);
    std::string opt_object = " --obj " + object;
    std::string opt_animation = " --anim " + animation;
    std::string opt_resolution = " --res " + std::to_string(res_x) + " " + std::to_string(res_y);
    std::string opt_size = " --mp " +  std::to_string(mp1) + " " +  std::to_string(mp2) + " " +  std::to_string(mp3);

    // DONT CHANGE
    std::string tpl_img = " --tpl_img " + mPkgPath + "/img/template_image.jpg";
    std::string bgr_img = " --bgr_img " + mPkgPath + "/img/background_image.jpg";
    std::string output  = " --out "
            + mPkgPath + "/render/video.avi";

    execute_script(py_file
                   + opt_frames
                   + opt_fps
                   + opt_object
                   + opt_animation
                   + opt_resolution
                   + opt_size
                   + tpl_img
                   + bgr_img
                   + output);
}

void PythonCaller::get_template_image_list(int length, int min_height, int min_width, std::string keywords)
{
    std::string py_file = "generate_image_list.py";
    std::string opt_length = " --length " + std::to_string(length);
    std::string opt_min_size = " --size " + std::to_string(min_width) + " " + std::to_string(min_height);
    std::string path = " --path " + mPkgPath + "/img/template_image_list.txt";

    execute_script(py_file + opt_length + opt_min_size + path + " --keywords " + keywords);
}

void PythonCaller::get_background_image_list(int length, int min_height, int min_width, std::string keywords)
{
    std::string py_file = "generate_image_list.py";

    std::string opt_length = " --length " + std::to_string(length);
    std::string opt_min_size = " --size " + std::to_string(min_width) + " " + std::to_string(min_height);
    std::string path = " --path " + mPkgPath + "/img/background_image_list.txt";

    execute_script(py_file + opt_length + opt_min_size + path + " --keywords " + keywords);
}

void PythonCaller::download_template_image(int nr)
{
    std::string py_file = "download_image.py";

    std::string path = " --path " + mPkgPath + "/img/template_image.jpg";
    std::string img_list = " --img_list " + mPkgPath + "/img/template_image_list.txt";

    execute_script(py_file + path + img_list + " --image_nr " + std::to_string(nr));
}

void PythonCaller::download_background_image(int nr)
{
    std::string py_file = "download_image.py";

    std::string path = " --path " + mPkgPath + "/img/background_image.jpg";
    std::string img_list = " --img_list " + mPkgPath + "/img/background_image_list.txt";

    execute_script(py_file + path + img_list + " --image_nr " + std::to_string(nr));
}

void PythonCaller::get_ground_truth_data(std::string animation, int frames, std::string path)
{
     std::string py_file = "get_ground_truth.py";

     std::string opt_path = " --path " + path;
     std::string opt_anim = " --anim " + animation;
     std::string opt_frames = " --frames " + std::to_string(frames);

     execute_script(py_file + opt_path + opt_anim + opt_frames);
}

void PythonCaller::get_init_pose(std::string animation, Pose &pose)
{
    std::string py_file = "get_init_pose.py";
    std::string opt_anim = " --anim " + animation;

    FILE* out = execute_script(py_file + opt_anim, 3);
    fscanf(out, "%lf %lf %lf %lf %lf %lf", &pose.x, &pose.y, &pose.z, &pose.ax, &pose.ay, &pose.az);

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
