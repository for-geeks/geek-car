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
#include <queue>
#include <opencv2/opencv.hpp>
#include "cyber/cyber.h"
#include "cyber/task/task.h"
#include "modules/sensors/proto/sensors.pb.h"
std::queue<std::shared_ptr<apollo::sensors::Image>> image_queue;

void Action() {
  const std::shared_ptr<apollo::sensors::Image> image = image_queue.front();
  image_queue.pop();

  cv::Mat new_image = cv::Mat(816, 848, CV_8U, (void*)(image->mutable_data()));
  std::string image_name = "/home/raosiyue/out_test/G_" +
                           std::to_string(image->frame_no()) + ".jpg";

  //cv::imshow("callback", new_image);
  cv::imwrite(image_name, new_image);

  ADEBUG << "Saved image :" << image_name;
  //int key = cv::waitKey(1);
  //if (key == 'q') {
  //  exit(0);
  //}
}

void ImageCallback(const std::shared_ptr<apollo::sensors::Image>& image) {
  ADEBUG << "image, height :" << image->height() << " width:" << image->width();
#if 1
  cv::Mat new_image = cv::Mat(static_cast<int>(image->height()),
                              static_cast<int>(image->width()), CV_8U,
                              (void*)image->data().c_str());
#endif
  //image_queue.push(image);
  //std::async(std::launch::async, Action);
#if 1
  std::string image_name = "/home/raosiyue/out_test/Gray_Image_" +
                           std::to_string(image->frame_no()) + ".jpg";

  //cv::imshow("callback", new_image);
  cv::imwrite(image_name, new_image);

  ADEBUG << "Saved image :" << image_name;
#endif
}


int main() {
  apollo::cyber::Init("image_save");
  auto node = apollo::cyber::CreateNode("image_save");
  auto reader = node->CreateReader<apollo::sensors::Image>(
      "/realsense/raw_image", ImageCallback);

  apollo::cyber::WaitForShutdown();

  return 0;
}
