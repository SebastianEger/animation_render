#include "imageevaluation.h"

ImageEvaluation::ImageEvaluation()
{

}

bool ImageEvaluation::evaluateTemplate(std::string path)
{
    cv::Mat gradX, gradY;

    // Read Template Image
    mTmplImage = cv::imread(path);
    mHelperMat = cv::Mat::zeros(mTmplImage.rows, mTmplImage.cols, CV_8U);

    cv::cvtColor(mTmplImage, mTmplImageGray, CV_BGR2GRAY);

    cv::Sobel( mTmplImageGray, gradX, CV_64F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_64F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    //cv::boxFilter(gradX, gradX, -1, cv::Size(100, 100), cv::Point(-1, -1), false);
    //cv::boxFilter(gradY, gradY, -1, cv::Size(100, 100), cv::Point(-1, -1), false);

    for(int x = 0; x < gradX.cols; x++) {
        for(int y = 0; y < gradX.rows; y++) {
            mHelperMat.at<uchar>(y, x) = checkGradientArea(gradX, x, y, 2);
        }
    }
    cv::imshow("Template Image", mTmplImageGray);
    cv::imshow("GradX", gradX);
    cv::imshow("GradX filtered", mHelperMat);
    cv::waitKey();
}

bool ImageEvaluation::showImage(std::string path)
{
    cv::imshow("Image", cv::imread(path));
    cv::waitKey();
}

int ImageEvaluation::checkGradientArea(cv::Mat &grad, double x, double y, int size)
{
    for(int i = -size; i < size+1; i++) {
        for(int j = -size; j < size+1; j++) {
            if(y + i < 0 || y + i > grad.rows || x + j < 0 || x + j >= grad.cols) continue;
            if( abs(grad.at<uchar>(y+i, x+j) ) > 20) return 255;
        }
    }
    return 0;
}
