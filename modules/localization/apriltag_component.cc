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
#include "modules/localization/apriltag_component.h"

#include <float.h>
#include <math.h>
#include <string>
#include <vector>

#include "apriltag_pose.h"
#include "opencv2/opencv.hpp"
#include "tag36h11.h"

#include "modules/common/global_gflags.h"

namespace apollo {
namespace localization {

// rotation matrix to euler angles
// https://www.learnopencv.com/rotation-matrix-to-euler-angles/
void rotation2euler_angle(matd_t* R) {
  double r21 = MATD_EL(R, 2, 1);
  double r22 = MATD_EL(R, 2, 2);
  double sy = sqrt(r21 * r21 + r22 * r22);

  double theta_x = atan2(MATD_EL(R, 2, 1), MATD_EL(R, 2, 2));
  double theta_y = atan2(-MATD_EL(R, 2, 0), sy);
  double theta_z = atan2(MATD_EL(R, 1, 0), MATD_EL(R, 0, 0));

  ADEBUG << "euler angles is, x:" << theta_x << " y:" << theta_y
         << " z:" << theta_z;
}

// conversion  matrix pointer to proto
void matd_t2proto(matd_t* mat, Matrix* r) {
  for (size_t row = 0; row < mat->nrows; ++row) {
    for (size_t c = 0; c < mat->ncols; ++c) {
      r->add_element(MATD_EL(mat, row, c));
    }
  }
}

bool ApriltagComponent::Init() {
  td_ = apriltag_detector_create();
  tf_ = tag36h11_create();

  apriltag_detector_add_family(td_, tf_);

  std::string image_to_detect = FLAGS_use_compressed_image_to_detect_tag
                                    ? FLAGS_compressed_gray_image_channel
                                    : FLAGS_gray_image_channel;

  image_reader_ = node_->CreateReader<Image>(
      image_to_detect, [this](const std::shared_ptr<Image>& image) {
        this->ApriltagDetection(image);
      });

  tags_writer_ = node_->CreateWriter<Tags>(FLAGS_tags_channel);

  return true;
}

void ApriltagComponent::ApriltagDetection(const std::shared_ptr<Image>& image) {
  ADEBUG << "Read image: in call back:" << image->frame_no()
         << " height:" << image->height() << " width:" << image->width();
  // TODO(all) config
  td_->quad_decimate = 2.0;
  td_->quad_sigma = 0.0;
  td_->refine_edges = 1;
  td_->decode_sharpening = 0.25;

  cv::Mat new_image;

  if (FLAGS_use_compressed_image_to_detect_tag) {
    std::vector<uchar> buff(
        (unsigned char*)image->data().c_str(),
        (unsigned char*)image->data().c_str() + image->data().length());
    new_image = cv::imdecode(buff, CV_8U);
  } else {
    new_image = cv::Mat(static_cast<int>(image->height()),
                        static_cast<int>(image->width()), CV_8U,
                        const_cast<char*>(image->data().c_str()));
  }

  image_u8_t im = {.width = new_image.cols,
                   .height = new_image.rows,
                   .stride = new_image.cols,
                   .buf = new_image.data};

  zarray_t* detections = apriltag_detector_detect(td_, &im);
  ADEBUG << "detected Apriltags:" << zarray_size(detections);

  auto tags = std::make_shared<Tags>();

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);

    // Do something with det here
    printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n", i,
           det->family->nbits, det->family->h, det->id, det->hamming,
           det->decision_margin);
    apriltag_detection_info_t info;
    info.det = det;
    info.tagsize = FLAGS_tagsize;
    info.fx = FLAGS_left_fx;
    info.fy = FLAGS_left_fy;
    info.cx = FLAGS_left_cx;
    info.cy = FLAGS_left_cy;

    apriltag_pose_t pose;
    double err = estimate_tag_pose(&info, &pose);
    ADEBUG << "detect err:" << err;
    ADEBUG << "estimate tag pose : R, ";
    matd_print(pose.R, "%15f");
    ADEBUG << "estimate tag pose : t, ";
    matd_print(pose.t, "%15f");

    rotation2euler_angle(pose.R);

    Tag tag;
    tag.set_id(det->id);
    tag.set_hamming(det->hamming);
    tag.set_margin(det->decision_margin);

    auto family = tag.mutable_family();
    family->set_nbits(det->family->nbits);
    family->set_h(det->family->h);

    auto r = tag.mutable_pose()->mutable_r();
    r->set_rows(pose.R->nrows);
    r->set_cols(pose.R->ncols);
    for (size_t row = 0; row < pose.R->nrows; ++row) {
      for (size_t c = 0; c < pose.R->ncols; ++c) {
        r->add_element(MATD_EL(pose.R, row, c));
      }
    }

    auto t = tag.mutable_pose()->mutable_t();
    t->set_rows(pose.t->nrows);
    t->set_cols(pose.t->ncols);

    for (size_t r = 0; r < pose.t->nrows; ++r) {
      for (size_t c = 0; c < pose.t->ncols; ++c) {
        t->add_element(MATD_EL(pose.t, r, c));
      }
    }

    auto p_proto = tag.mutable_pose();
    p_proto->set_error(err);

    auto next_tag = tags->add_tag();
    next_tag->CopyFrom(tag);
  }
  apriltag_detections_destroy(detections);
  tags_writer_->Write(tags);
}

ApriltagComponent::~ApriltagComponent() {
  if (image_ready_.load()) {
    // close
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }
}

}  // namespace localization
}  // namespace apollo
