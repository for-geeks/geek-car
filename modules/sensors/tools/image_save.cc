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
#include "cyber/cyber.h"
#include "cyber/task/task.h"
#include "modules/common/global_gflags.h"
#include "modules/sensors/proto/sensor_image.pb.h"
#include "modules/sensors/proto/sensors.pb.h"
#include "opencv2/opencv.hpp"

void ImageCallback(
    const std::shared_ptr<apollo::sensors::CompressedImage>& image) {
  // ADEBUG << "image, height :" << image->height() << " width:" <<
  // image->width();
  std::vector<uchar> buff(
      (unsigned char*)image->data().c_str(),
      (unsigned char*)image->data().c_str() + image->data().length());

  cv::Mat new_image = cv::imdecode(buff, CV_8UC2);
  // cv::Mat new_image = cv::Mat(static_cast<int>(image->height()),
  //                            static_cast<int>(image->width()), CV_8UC3,
  //                            const_cast<char*>(image->data().c_str()));
  std::string image_name =
      FLAGS_image_export_dir + std::to_string(image->frame_no()) + ".jpg";
  cv::imwrite(image_name, new_image);
  ADEBUG << "Saved image :" << image_name;
}

int main() {
  apollo::cyber::Init("image_save");
  auto node = apollo::cyber::CreateNode("image_save");
  auto reader = node->CreateReader<apollo::sensors::CompressedImage>(
      FLAGS_compressed_color_image_channel, ImageCallback);

  apollo::cyber::WaitForShutdown();

  return 0;
}
