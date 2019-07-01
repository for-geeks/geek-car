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
#include "modules/perception/perception_component.h"

namespace apollo {
namespace perception {

using apollo::sensors::Image;

bool PerceptionComponent::Init() {
  // maybe some config here

  return true;
}

bool PerceptionComponent::Proc(const std::shared_ptr<Image>& image) {
  ADEBUG << image->DebugString();

  // cv::Mat new_image =
  // (static_cast<int>(image->height()), static_cast<int>(image->width()),
  // CV_8U,
  //  (void*)image->mutable_data());

  // std::string image_name = "/home/raosiyue/out_test/Gray_Image_" +
  // std::to_string(image->frame_no()) + ".jpg ";

  // cv::imshow("callback", new_image);
  // cv::imwrite(image_name, new_image);

  // ADEBUG << "Saved image :" << image_name;

  return true;
}

}  // namespace perception
}  // namespace apollo
