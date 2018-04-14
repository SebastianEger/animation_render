#ifndef IMAGEEVALUATION_H
#define IMAGEEVALUATION_H

#include <string>
#include <opencv2/opencv.hpp>

class ImageEvaluation
{
public:
    ImageEvaluation();

    bool evaluateTemplate(std::string path);
    static bool showImage(std::string path);

private:
    cv::Mat mTmplImage;
    cv::Mat mTmplImageGray;
    cv::Mat mHelperMat;

    int checkGradientArea(cv::Mat &grad, double x, double y, int size);
};

#endif // IMAGEEVALUATION_H
