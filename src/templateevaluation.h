#ifndef TEMPLATEEVALUATION_H
#define TEMPLATEEVALUATION_H


#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <boost/filesystem.hpp>

class TemplateEvaluation
{
public:
    TemplateEvaluation();
    TemplateEvaluation(double t_grad, double t_acc);

    bool evaluate(std::string file);
    bool evaluate(std::string file, double &acc_x, double &acc_y);

    bool evaluate2(std::string file);

    bool evaluate3(std::string file, bool out=false);

    bool evaluate4(std::string file, bool out=false);

    bool evaluate5(std::string file, bool out=false);

    static bool showImage(std::string path);

    double mGradThreshold;
    double mAcceptanceThreshold;

    void exportData(std::string path, std::string file);

private:
    cv::Mat mTmplImage;
    cv::Mat mTmplImageGray;
    cv::Mat mHelperMat;


    std::vector<double> mDataX;
    std::vector<double> mDataY;

    std::vector<double> mTemplateDataX;
    std::vector<double> mTemplateDataY;
};

#endif // TEMPLATEEVALUATION_H
