#include <opencv2/opencv.hpp>
#include "cyber/cyber.h"
#include "modules/sensors/proto/sensors.pb.h"

void ImageCallback(const std::shared_ptr<apollo::sensors::Image>& image) {
  ADEBUG << "image, height :" << image->height() << " width:" << image->width();

  cv::Mat new_image =
      (static_cast<int>(image->height()), static_cast<int>(image->width()),
       CV_8U, (void*)image->mutable_data());

  std::string image_name = "/home/raosiyue/out_test/Gray_Image_" +
                           std::to_string(image->frame_no()) + ".jpg ";

  cv::imshow("callback", new_image);
  cv::imwrite(image_name, new_image);

  ADEBUG << "Saved image :" << image_name;

  // Origin reference : conversion from bytes to MAT
  //   Mat bytesToMat(byte * bytes, int width, int height) {
  //     Mat image = Mat(height, width, CV_8UC3, bytes).clone();  // make a copy
  //     return image;
  //   }

  int key = cv::waitKey(1);
  if (key == 'q') {
    exit(0);
  }
}

int main() {
  apollo::cyber::Init("image_save");
  auto node = apollo::cyber::CreateNode("image_save");
  auto reader = node->CreateReader<apollo::sensors::Image>(
      "/realsense/raw_image", ImageCallback);

  apollo::cyber::WaitForShutdown();

  return 0;
}
