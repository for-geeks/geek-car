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
#include "apriltag.h"
#include "librealsense2/rs.hpp"
#include "tag36h11.h"

#include "modules/common/global_gflags.h"

namespace apollo {
namespace localization {

using apollo::localization::Tags;

inline rs2_quaternion quaternion_exp(rs2_vector v) {
  float x = v.x / 2, y = v.y / 2, z = v.z / 2, th2,
        th = sqrtf(th2 = x * x + y * y + z * z);
  float c = cosf(th),
        s = th2 < sqrtf(120 * FLT_EPSILON) ? 1 - th2 / 6 : sinf(th) / th;
  rs2_quaternion Q = {s * x, s * y, s * z, c};
  return Q;
}

inline rs2_quaternion quaternion_multiply(rs2_quaternion a, rs2_quaternion b) {
  rs2_quaternion Q = {
      a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
      a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
      a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
  return Q;
}

rs2_pose predict_pose(rs2_pose& pose, float dt_s) {
  rs2_pose P = pose;
  P.translation.x = dt_s * (dt_s / 2 * pose.acceleration.x + pose.velocity.x) +
                    pose.translation.x;
  P.translation.y = dt_s * (dt_s / 2 * pose.acceleration.y + pose.velocity.y) +
                    pose.translation.y;
  P.translation.z = dt_s * (dt_s / 2 * pose.acceleration.z + pose.velocity.z) +
                    pose.translation.z;
  rs2_vector W = {
      dt_s * (dt_s / 2 * pose.angular_acceleration.x + pose.angular_velocity.x),
      dt_s * (dt_s / 2 * pose.angular_acceleration.y + pose.angular_velocity.y),
      dt_s * (dt_s / 2 * pose.angular_acceleration.z + pose.angular_velocity.z),
  };
  P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
  return P;
}

bool LocalizationComponent::Init() {
  // pose_reader_ = node_->CreateReader<Pose>(
  //     FLAGS_pose_channel, [this](const std::shared_ptr<Pose>& pose) {
  //       predicted_pose_ = predict_pose(pose);
  //     });

  image_reader_ = node_->CreateReader<Image>(
      FLAGS_raw_image_channel, [this](const std::shared_ptr<Image>& image) {
        image_ = image;
        // tell tag detection you can work now
        image_ready_.exchange(true);
      });

  tags_writer_ = node_->CreateWriter<Tags>(FLAGS_tags_channel);

  tag_async_ = std::async(std::launch::async,
                          &LocalizationComponent::ApriltagDetection, this);

  return true;
}

void LocalizationComponent::ApriltagDetection() {
  // if image not ready, exit directly
  if (!image_ready_.load()) {
    return;
  }
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_family_t* tf = tag36h11_create();
  apriltag_detector_add_family(td, tf);

  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;

  zarray_t* detections = apriltag_detector_detect(td, image_);

  if (!zarray_size(detections)) {
    return;
  }

  auto tags = std::make_ptr<Tags>();

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);

    // Do something with det here
    printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n", i,
           det->family->nbits, det->family->h, det->id, det->hamming,
           det->decision_margin);
    auto tag = tags->mutable_tag();

    tag->set_id(det->id);
    tag->set_hamming(det->hamming);
    tag->set_margin(det->decision_margin);

    auto family = tag->mutable_family();
    family->set_nbits(det->family->nbits);
    family->set_h(det->family->h);
  }

  tag_writer_->Write(tags);

  apriltag_detections_destroy(detections);

  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
}

LocalizationComponent::~LocalizationComponent() {
  if (image_ready_.load()) {
    // close
  }
}

}  // namespace localization
}  // namespace apollo
