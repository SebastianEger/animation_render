#ifndef TEMPLATEEVALUATION_H
#define TEMPLATEEVALUATION_H


#include <string>
#include <opencv2/opencv.hpp>

class TemplateEvaluation
{
public:
    TemplateEvaluation();
    TemplateEvaluation(double t_grad, double t_acc);

    bool evaluate(std::string file);
    bool evaluate(std::string file, double &acc_x, double &acc_y);
    static bool showImage(std::string path);

    double mGradThreshold;
    double mAcceptanceThreshold;

private:
    cv::Mat mTmplImage;
    cv::Mat mTmplImageGray;
    cv::Mat mHelperMat;
};

#endif // TEMPLATEEVALUATION_H
