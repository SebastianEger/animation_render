#include<opencv2/opencv.hpp>

int main( int argc, char** argv )
{
    std::string file = std::string(argv[1]);
    cv::Mat image = cv::Mat::zeros(512, 512, CV_8UC1);
    int intensity = 255;

    for(int i = 0; i < 256; i++) {
        int x = 0;
        int y = i;
        while( y >= 0) {
            image.at<uchar>(y, x) = intensity;
            x++;
            y--;
        }
        intensity--;
    }

    intensity = 255;
    for(int i = 255; i < 512; i++) {
        int x = 255;
        int y = i;
        while( y >= 255) {
            image.at<uchar>(y, x) = intensity;
            x++;
            y--;
        }
        intensity--;
    }

    intensity = 255;
    for(int i = 511; i > 255; i--) {
        int x = 0;
        int y = i;
        while( y <= 511) {
            image.at<uchar>(y, x) = intensity;
            x++;
            y++;
        }
        intensity--;
    }

    intensity = 255;
    for(int i = 255; i >= 0; i--) {
        int x = 255;
        int y = i;
        while( y <= 255) {
            image.at<uchar>(y, x) = intensity;
            x++;
            y++;
        }
        intensity--;
    }

    intensity = 255;
    for(int i = 0; i < 256; i++) {
        int x = 511;
        int y = i;
        while( y >= 0 ) {
            image.at<uchar>(y, x) = intensity;
            x--;
            y--;
        }
        intensity--;
    }

    intensity = 255;
    for(int i = 0; i < 256; i++) {
        int x = 511;
        int y = i;
        while( y >= 0 ) {
            image.at<uchar>(y, x) = intensity;
            x--;
            y--;
        }
        intensity--;
    }

    intensity = 255;
    for(int i = 255; i < 512; i++) {
        int x = 255;
        int y = i;
        while( y >= 255 ) {
            image.at<uchar>(y, x) = intensity;
            x--;
            y--;
        }
        intensity--;
    }


    intensity = 255;
    for(int i = 511; i > 255; i--) {
        int x = 511;
        int y = i;
        while( y <= 511 ) {
            image.at<uchar>(y, x) = intensity;
            x--;
            y++;
        }
        intensity--;
    }

    intensity = 255;
    for(int i = 255; i >= 0; i--) {
        int x = 255;
        int y = i;
        while( y <= 255 ) {
            image.at<uchar>(y, x) = intensity;
            x--;
            y++;
        }
        intensity--;
    }

    std::default_random_engine generator;
    // int dice_roll = distribution(generator);

    for(int y = 0; y < 512; y++) {
        std::uniform_int_distribution<int> distribution(y, y+50);
        for(int x = 0; x < 521; x++) {
            image.at<uchar>(y, x) = distribution(generator);
        }
    }
    cv::imwrite( "./template.jpg", image );
    return 0;
}
