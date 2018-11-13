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

    cv::Sobel( mTmplImageGray, gradX, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    int gradXcounter = 0;
    int gradYcounter = 0;
    for(int x = 0; x < gradX.cols; x++) {
        for(int y = 0; y < gradX.rows; y++) {
            if(fabs(gradX.at<float>(y, x)) > mGradThreshold) gradXcounter ++;
            if(fabs(gradY.at<float>(y, x)) > mGradThreshold) gradYcounter ++;
        }
    }

    if(gradXcounter / double(gradX.cols*gradX.rows) > mAcceptanceThreshold && gradYcounter / double(gradY.cols*gradY.rows) > mAcceptanceThreshold) {
        ROS_INFO_STREAM(file << " did pass evaluation. X: " << std::to_string(gradXcounter / double(gradX.cols*gradX.rows))
                        << " Y: " << std::to_string(gradYcounter / double(gradY.cols*gradY.rows)) );
        return true;
    } else {
        ROS_INFO_STREAM(file << " didnt pass evaluation. X: " << std::to_string(gradXcounter / double(gradX.cols*gradX.rows))
                        << " Y: " << std::to_string(gradYcounter / double(gradY.cols*gradY.rows)) );
        return false;
    }
}

// ratio of gradients over threshold over image size (pixel number)
bool TemplateEvaluation::evaluate(std::string file, double &acc_x, double &acc_y)
{
    cv::Mat gradX, gradY;

    // Read Template Image
    mTmplImage = cv::imread(file);
    mHelperMat = cv::Mat::zeros(mTmplImage.rows, mTmplImage.cols, CV_8U);

    cv::cvtColor(mTmplImage, mTmplImageGray, CV_BGR2GRAY);

    cv::Sobel( mTmplImageGray, gradX, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    int gradXcounter = 0;
    int gradYcounter = 0;
    for(int x = 0; x < gradX.cols; x++) {
        for(int y = 0; y < gradX.rows; y++) {
            if(abs(gradX.at<float>(y, x)) > mGradThreshold) gradXcounter ++;
            if(abs(gradY.at<float>(y, x)) > mGradThreshold) gradYcounter ++;
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

// sum of gradients over image size (pixel number)
bool TemplateEvaluation::evaluate2(std::string file)
{
    cv::Mat gradX, gradY;

    // Read Template Image
    mTmplImage = cv::imread(file);
    mHelperMat = cv::Mat::zeros(mTmplImage.rows, mTmplImage.cols, CV_8U);

    cv::cvtColor(mTmplImage, mTmplImageGray, CV_BGR2GRAY);

    cv::Sobel( mTmplImageGray, gradX, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    double sumGradX, sumGradY;
    for(int x = 0; x < gradX.cols; x++) {
        for(int y = 0; y < gradX.rows; y++) {
            sumGradX += abs(gradX.at<float>(y, x));
            sumGradY += abs(gradY.at<float>(y, x));
        }
    }
}

// moving window over image, ratio gradient over threshold
bool TemplateEvaluation::evaluate3(std::string file, bool out)
{
    cv::Mat gradX, gradY;

    // Read Template Image
    mTmplImage = cv::imread(file);
    mHelperMat = cv::Mat::zeros(mTmplImage.rows, mTmplImage.cols, CV_8U);

    cv::cvtColor(mTmplImage, mTmplImageGray, CV_BGR2GRAY);

    cv::Sobel( mTmplImageGray, gradX, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    double win_counter = 0;
    for(int i = 0; i < gradX.rows; i++) {
        int win_start = i - gradX.rows / 2;

        int gradXcounter = 0;
        int gradYcounter = 0;
        for(int y = 0; y < gradX.rows / 2; y++) {
            int row = y + win_start;
            if(row < 0) row += gradX.rows;
            for(int x = 0; x < gradX.cols; x++) {
                if(abs(gradX.at<float>(row, x)) > mGradThreshold) gradXcounter ++;
                if(abs(gradY.at<float>(row, x)) > mGradThreshold) gradYcounter ++;
            }
        }

        double ratio_x = gradXcounter / double(gradX.cols*gradX.rows/2.0);
        double ratio_y = gradYcounter / double(gradY.cols*gradY.rows/2.0);

        if(out) ROS_INFO_STREAM("W" << std::to_string(i) << " X: " << std::to_string(ratio_x) << " Y: " << std::to_string(ratio_y));
        if(ratio_x > mGradThreshold && ratio_y > mGradThreshold) win_counter++;
    }
    if(out) ROS_INFO_STREAM(file << " - Window Acceptance Rate: " << std::to_string(win_counter / gradX.rows));
}

bool TemplateEvaluation::evaluate4(std::string file, bool out)
{
    cv::Mat gradX, gradY;

    // Read Template Image
    mTmplImage = cv::imread(file);
    mHelperMat = cv::Mat::zeros(mTmplImage.rows, mTmplImage.cols, CV_8U);

    cv::GaussianBlur(mTmplImage, mTmplImage, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
    cv::cvtColor(mTmplImage, mTmplImageGray, CV_BGR2GRAY);

    cv::Sobel( mTmplImageGray, gradX, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    double win_counter = 0;
    mDataX.clear();
    mDataY.clear();

    for(int i = 0; i < gradX.rows; i++) {
        int win_start = i - gradX.rows / 2;

        double sumX = 0;
        double sumY = 0;
        for(int y = 0; y < gradX.rows / 2; y++) {
            int row = y + win_start;
            if(row < 0) row += gradX.rows;
            for(int x = 0; x < gradX.cols; x++) {
                sumX += abs(gradX.at<float>(row, x));
                sumY += abs(gradY.at<float>(row, x));
            }
        }

        double ratio_x = sumX / double(gradX.cols*gradX.rows/2.0);
        double ratio_y = sumY / double(gradY.cols*gradY.rows/2.0);

        mDataX.push_back(ratio_x);
        mDataY.push_back(ratio_y);
        if(out) ROS_INFO_STREAM("W" << std::to_string(i) << " X: " << std::to_string(ratio_x) << " Y: " << std::to_string(ratio_y));
        if(ratio_x >= mGradThreshold && ratio_y >= mGradThreshold) win_counter++;
    }

    if(win_counter / gradX.rows >= mAcceptanceThreshold) {
        ROS_INFO_STREAM(file << " PASSED - WAR: " << std::to_string(win_counter / gradX.rows));
        return true;
    } else {
        ROS_INFO_STREAM(file << " FAILED - WAR: " << std::to_string(win_counter / gradX.rows));
        return false;
    }
}

bool TemplateEvaluation::evaluate5(std::string file, bool out)
{
    cv::Mat gradX, gradY;

    // Read Template Image
    mTmplImage = cv::imread(file);
    mHelperMat = cv::Mat::zeros(mTmplImage.rows, mTmplImage.cols, CV_8U);

    cv::cvtColor(mTmplImage, mTmplImageGray, CV_BGR2GRAY);

    cv::Sobel( mTmplImageGray, gradX, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( mTmplImageGray, gradY, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    double win_counter = 0;

    mTemplateDataX.clear();
    mTemplateDataY.clear();
    for(int y = 0; y <gradX.rows; y++) {
        for(int x = 0; x < gradX.cols; x++) {
            mTemplateDataX.push_back(gradX.at<float>(y, x));
            mTemplateDataY.push_back(gradY.at<float>(y, x));
        }
    }
    mDataX.clear();
    mDataY.clear();

    for(int i = 0; i < gradX.rows; i++) {
        int win_start = i - gradX.rows / 2;

        double gradXcounter = 0;
        double gradYcounter = 0;
        double win_total  = 0;
        for(int y = 0; y < gradX.rows / 2; y++) {
            int row = y + win_start;
            if(row < 0) row += gradX.rows;
            for(int x = 0; x < gradX.cols; x++) {
                if(fabs(gradX.at<float>(row, x)) > mGradThreshold) gradXcounter ++;
                if(fabs(gradY.at<float>(row, x)) > mGradThreshold) gradYcounter ++;
                ++win_total;
            }
        }

        double ratio_x = gradXcounter / win_total;
        double ratio_y = gradYcounter / win_total;

        mDataX.push_back(ratio_x);
        mDataY.push_back(ratio_y);
        if(out) ROS_INFO_STREAM("W" << std::to_string(i) << " X: " << std::to_string(ratio_x) << " Y: " << std::to_string(ratio_y));
        if(ratio_x >= mAcceptanceThreshold && ratio_y >= mAcceptanceThreshold) win_counter++;
    }

    if(win_counter / double(gradX.rows) >= 0.9) {
        ROS_INFO_STREAM(file << " PASSED - WAR: " << std::to_string(win_counter / gradX.rows));
        return true;
    } else {
        ROS_INFO_STREAM(file << " FAILED - WAR: " << std::to_string(win_counter / gradX.rows));
        return false;
    }
}

bool TemplateEvaluation::showImage(std::string path)
{
    cv::imshow("Image", cv::imread(path));
    cv::waitKey();
}

void TemplateEvaluation::exportData(std::string path, std::string file)
{
    const char* p = path.c_str();
    boost::filesystem::path dir(p);
    if(boost::filesystem::create_directories(dir))
    {
        ROS_INFO_STREAM("Directory Created: " << path);
    }

    ROS_INFO_STREAM("Exporting data to: " << path);
    path += "/";

    std::ofstream ofs;
    ofs.open (path + file, std::ofstream::out);
    for(std::vector<double>::iterator it = mDataX.begin(); it != mDataX.end(); it++) {
        ofs << *it << " ";
    }
    ofs << std::endl;
    for(std::vector<double>::iterator it = mDataY.begin(); it != mDataY.end(); it++) {
        ofs << *it << " ";
    }
    ofs << std::endl;
    ofs.close();

    ofs.open (path + file + "_grad", std::ofstream::out);
    for(std::vector<double>::iterator it = mTemplateDataX.begin(); it != mTemplateDataX.end(); it++) {
        ofs << *it << " ";
    }
    ofs << std::endl;
    for(std::vector<double>::iterator it = mTemplateDataY.begin(); it != mTemplateDataY.end(); it++) {
        ofs << *it << " ";
    }
    ofs << std::endl;
    ofs.close();
}
