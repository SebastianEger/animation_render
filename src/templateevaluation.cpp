#include "templateevaluation.h"

TemplateEvaluation::TemplateEvaluation()
{

}

TemplateEvaluation::TemplateEvaluation(double t_grad, double t_acc) :
    mGradThreshold(t_grad),
    mAcceptanceThreshold(t_acc)
{

}

bool TemplateEvaluation::evaluate(std::string file)
{
    cv::Mat gradX, gradY;

    // Read Template Image
    mTmplImage = cv::imread(file);
    mHelperMat = cv::Mat::zeros(mTmplImage.rows, mTmplImage.cols, CV_8U);

    cv::cvtColor(mTmplImage, mTmplImageGray, CV_BGR2GRAY);

    cv::Sobel( mTmplImageGray, gradX, CV_64F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_64F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    int gradXcounter = 0;
    int gradYcounter = 0;
    for(int x = 0; x < gradX.cols; x++) {
        for(int y = 0; y < gradX.rows; y++) {
            if(abs(gradX.at<uchar>(y, x)) > mGradThreshold) gradXcounter ++;
            if(abs(gradY.at<uchar>(y, x)) > mGradThreshold) gradYcounter ++;
        }
    }

    if(gradXcounter / double(gradX.cols*gradX.rows) > mAcceptanceThreshold && gradYcounter / double(gradY.cols*gradY.rows) > mAcceptanceThreshold) {
        return true;
    } else {
        return false;
    }
}

bool TemplateEvaluation::evaluate(std::string file, double &acc_x, double &acc_y)
{
    cv::Mat gradX, gradY;

    // Read Template Image
    mTmplImage = cv::imread(file);
    mHelperMat = cv::Mat::zeros(mTmplImage.rows, mTmplImage.cols, CV_8U);

    cv::cvtColor(mTmplImage, mTmplImageGray, CV_BGR2GRAY);

    cv::Sobel( mTmplImageGray, gradX, CV_64F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_64F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    int gradXcounter = 0;
    int gradYcounter = 0;
    for(int x = 0; x < gradX.cols; x++) {
        for(int y = 0; y < gradX.rows; y++) {
            if(abs(gradX.at<uchar>(y, x)) > mGradThreshold) gradXcounter ++;
            if(abs(gradY.at<uchar>(y, x)) > mGradThreshold) gradYcounter ++;
        }
    }

    acc_x = gradXcounter / double(gradX.cols*gradX.rows);
    acc_y = gradYcounter / double(gradY.cols*gradY.rows);

    if( acc_x > mAcceptanceThreshold && acc_y > mAcceptanceThreshold) {
        return true;
    } else {
        return false;
    }
}

bool TemplateEvaluation::showImage(std::string path)
{
    cv::imshow("Image", cv::imread(path));
    cv::waitKey();
}
