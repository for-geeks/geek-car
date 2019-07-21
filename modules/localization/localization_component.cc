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
#include "modules/localization/localization_component.h"

#include <float.h>
#include <math.h>
#include "apriltag_pose.h"
#include "opencv2/opencv.hpp"
#include "tag36h11.h"

#include "modules/common/global_gflags.h"

namespace apollo {
namespace localization {

using apollo::localization::Matrix;
using apollo::localization::Tag;
using apollo::localization::Tags;
using apollo::sensors::Image;

bool LocalizationComponent::Init() {
  // pose_reader_ = node_->CreateReader<Pose>(
  //     FLAGS_pose_channel, [this](const std::shared_ptr<Pose>& pose) {
  //       predicted_pose_ = predict_pose(pose);
  //     });

  image_reader_ = node_->CreateReader<Image>(
      FLAGS_raw_image_channel, [this](const std::shared_ptr<Image>& image) {
        image_.Clear();
        image_.CopyFrom(*image);
        ADEBUG << "Read image: in call back:" << image_.frame_no()
               << " height:" << image_.height() << " width:" << image_.width();
        // tell tag detection you can work now
        // image_ready_.exchange(true);

        // TODO(all) config
        td_->quad_decimate = 2.0;
        td_->quad_sigma = 0.0;
        td_->refine_edges = 1;
        td_->decode_sharpening = 0.25;

        cv::Mat new_image = cv::Mat(static_cast<int>(image_.height()),
                                    static_cast<int>(image_.width()), CV_8U,
                                    (void*)image_.data().c_str());
        image_u8_t im = {.width = new_image.cols,
                         .height = new_image.rows,
                         .stride = new_image.cols,
                         .buf = new_image.data};

        zarray_t* detections = apriltag_detector_detect(td_, &im);
        ADEBUG << "detected tags:" << zarray_size(detections);

        auto tags = std::make_shared<Tags>();

        for (int i = 0; i < zarray_size(detections); i++) {
          apriltag_detection_t* det;
          zarray_get(detections, i, &det);

          // Do something with det here
          printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                 i, det->family->nbits, det->family->h, det->id, det->hamming,
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
          ADEBUG << "estimate tag pose : R, ";
          matd_print(pose.R, "%15f");
          ADEBUG << "estimate tag pose : t, ";
          matd_print(pose.t, "%15f");

          Tag tag;
          tag.set_id(det->id);
          tag.set_hamming(det->hamming);
          tag.set_margin(det->decision_margin);

          auto family = tag.mutable_family();
          family->set_nbits(det->family->nbits);
          family->set_h(det->family->h);

          Matrix R, t;
          R->set_rows(pose.R.nrows);
          R->set_cols(pose.R.ncols);
          for (int r = 0; r < pose.R.nrows; ++r) {
            for (int c = 0; c < pose.R.ncols; ++c) {
              R->add_element(MATD_EL(pose.R, r, c));
            }
          }

          t->set_rows(pose.t.nrows);
          t->set_cols(pose.t.ncols);
          for (int r = 0; r < pose.t.nrows; ++r) {
            for (int c = 0; c < pose.t.ncols; ++r) {
              t->add_element(MATD_EL(pose.t, r, c));
            }
          }
          auto pose = tag.mutable_pose();
          pose->set_R(R);
          pose->set_t(t);

          auto next_tag = tags->add_tag();
          next_tag->CopyFrom(tag);
        }
        apriltag_detections_destroy(detections);
        tags_writer_->Write(tags);
      });

  tags_writer_ = node_->CreateWriter<Tags>(FLAGS_tags_channel);
  td_ = apriltag_detector_create();
  tf_ = tag36h11_create();

  apriltag_detector_add_family(td_, tf_);
  ApriltagDetection();
  return true;
}

void LocalizationComponent::ApriltagDetection() {
  while (!cyber::IsShutdown()) {
    if (!image_.has_data()) {
      ADEBUG << "IMAGE is not ready";
      return;
    }
    // if image not ready, exit directly
    ADEBUG << "image_ready_:" << image_ready_;
    ADEBUG << image_.DebugString();
    // if (!image_ready_.load()) {
    //  return;
    //}
  }
}

LocalizationComponent::~LocalizationComponent() {
  if (image_ready_.load()) {
    // close
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }
}

}  // namespace localization
}  // namespace apollo
