#include <opencv2/opencv.hpp>
#include "cyber/cyber.h"
#include "modules/sensors/proto/sensors.pb.h"

using apollo::cyber;

void ImageCallback(const std::shared_ptr<apollo::sensors::Image>& image) {
  ADEBUG << "image height :" << image.height() << " width:" << image.width();

  Mat image = Mat(image.height(), image.width(), CV_8U, image.data()).clone();

  std::string image_name =
      "/home/raosiyue/" + image.frame_no() + "Gray_Image.jpg";

  cv::imshow("callback", image);
  imwrite(image_name, image);

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
