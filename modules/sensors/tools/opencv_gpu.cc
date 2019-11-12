/******************************************************************************
 * MIT License

 * Copyright (c) 2019 Geekstyle

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
******************************************************************************/
#include <cstdlib>
#include <opencv2/gapi.hpp>
#include <opencv2/gapi/core.hpp>
#include <opencv2/gapi/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

static cv::GMat convertScaleAbs(const cv::GMat& src, double alpha = 1.0,
                                double beta = 0.0) {
  auto result =
      cv::gapi::absDiffC(cv::gapi::addC(cv::gapi::mulC(src, alpha), beta), 0.0);
  return cv::gapi::convertTo(result, CV_8UC1);
}

int main(int argc, char* argv[]) {
  cv::GMat gIn;

  auto imgBlur =
      cv::gapi::gaussianBlur(gIn, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

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
