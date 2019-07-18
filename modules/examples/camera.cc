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
// #include "opencv2/opencv.hpp"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/examples/proto/examples.pb.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::examples::proto::Image;

#if 0
std::string get_tegra_pipeline(int width, int height, int fps) {
  return "nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=1024, height=768, "
         "framerate=120/1, format=NV12' ! nvvidconv ! nvegltransform ! "
         "nveglglessink -e";
  // return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" +
  //        std::to_string(width) + ", height=(int)" + std::to_string(height) +
  //        ", format=(string)I420, framerate=(fraction)" + std::to_string(fps)
  //        +
  //        "/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! "
  //        "videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}
#endif
int main(int argc, char *argv[]) {
#if 0
  // Options
  const uint32_t WIDTH = 1024;
  const uint32_t HEIGHT = 768;
  // const uint32_t FPS = 30;

  // Define the gstream pipeline
  // std::string pipeline = get_tegra_pipeline(WIDTH, HEIGHT, FPS);
  // std::cout << "Using pipeline: \n\t" << pipeline << "\n";

  // Create OpenCV capture object, ensure it works.
  // cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
  // if (!cap.isOpened()) {
  //  std::cout << "Connection failed";
  //  return -1;
  //}

  cv::Mat origin_image = cv::imread("/home/nvidia/Pictures/2019-06-19.png");

  apollo::cyber::Init(argv[0]);

  auto image_node = apollo::cyber::CreateNode("image");
  auto image_writer = image_node->CreateWriter<Image>("/apollo/image");

  // View video
  cv::Mat frame;
  while (1) {
    // cap >> frame;  // Get a new frame from camera

    // Display frame
    imshow("Display window", origin_image);
    cv::waitKey(1);  // needed to show frame

    while (apollo::cyber::OK()) {
      auto image = std::make_shared<Image>();
      image->set_frame_id("1");
      image->set_measurement_time(23432.22);
      image->set_width(WIDTH);
      image->set_height(HEIGHT);
      auto m_size =
          origin_image.rows * origin_image.cols * origin_image.elemSize();
      image->set_data(origin_image.data, m_size);

      image_writer->Write(image);
    }
  }
#endif
  return 0;
}
