#include <cstdlib>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/core.hpp>
#include <opencv2/gapi/imgproc.hpp>

static cv::GMat convertScaleAbs(const cv::GMat & src, double alpha = 1.0, double beta = 0.0)
{
    auto result = cv::gapi::absDiffC(cv::gapi::addC(cv::gapi::mulC(src, alpha), beta), 0.0);
    return cv::gapi::convertTo(result, CV_8UC1);
}

int main(int argc, char * argv[])
{
    cv::GMat gIn;

    auto imgBlur = cv::gapi::gaussianBlur(gIn, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    auto imgGray = cv::gapi::convertTo(imgBlur, CV_32F);

    auto sobelX = cv::gapi::Sobel(imgGray, CV_16S, 1, 0, 3);
    auto sobelY = cv::gapi::Sobel(imgGray, CV_16S, 0, 1, 3);

    auto gradX = convertScaleAbs(sobelX);
    auto gradY = convertScaleAbs(sobelY);

    auto gOut = cv::gapi::addWeighted(sobelX, 0.5, sobelY, 0.5, 0);

    cv::GComputation computation(cv::GIn(gIn), cv::GOut(gOut));

    cv::Mat imgIn = cv::imread("/home/geek-car/out_test/402.jpg"), imgOut;

    computation.apply(cv::gin(imgIn), cv::gout(imgOut));

    cv::imshow("out.png", imgOut);

    return EXIT_SUCCESS;
}
