/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/tools/visualizer/video_image_viewer.h"

#include <iostream>

VideoImgViewer::VideoImgViewer(QWidget* parent)
    : QOpenGLWidget(parent),
      QOpenGLFunctions(),
      is_init_(false),
      mvp_id_(),
      plane_(),
      ortho_camera_(),
      default_image_(nullptr),
      video_image_shader_prog_(nullptr) {}

VideoImgViewer::~VideoImgViewer() {
  if (is_init_) {
    is_init_ = false;

    makeCurrent();

    plane_.Destroy();
    video_image_shader_prog_->destroyed();
    video_image_shader_prog_.reset();
    default_image_.reset();

    doneCurrent();
  }
}

void VideoImgViewer::initializeGL() {
  initializeOpenGLFunctions();

  glPointSize(1.0f);
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  QImage noImage;
  if (!noImage.load(tr(":/images/no_image.png"))) {
    std::cout << "--------can not load the default texture------------"
              << std::endl;
    return;
  }

  default_image_ = std::make_shared<Texture>();
  if (default_image_ == nullptr || !default_image_->UpdateData(noImage)) {
    std::cout << "--------can not create the default texture------------"
              << std::endl;
    return;
  }

    static const char *vertexShaderSource =
        "#version 130\n"
        "in vec2 vertPos;\n"
        "in vec2 texCoord;\n"
        "out vec2 TexCoord;\n"
        "void main() {\n"
        "   gl_Position = vec4(vertPos.x, vertPos.y, 0.0, 1.0);\n"
        "   TexCoord = vec2(texCoord.x, 1.0 - texCoord.y);\n"
        "}\n";



    static const char *fragmentShaderSource =
    "#version 130\n"
    "in mediump vec2 TexCoord;\n"
    "out mediump vec4 FragColor;\n"
    "uniform sampler2D texture;\n"
    "void main() {\n"
    "   FragColor = texture2D(texture, TexCoord);\n"
    "}\n";

  video_image_shader_prog_ = RenderableObject::CreateShaderProgram(
      vertexShaderSource,
      fragmentShaderSource);
  if (video_image_shader_prog_ == nullptr) {
    return;
  }

  plane_.set_texture(default_image_);
  if (!plane_.Init(video_image_shader_prog_)) {
    return;
  }

  ortho_camera_.set_near_plane_width(GLfloat(width()));
  ortho_camera_.set_near_plane_height(GLfloat(height()));

  ortho_camera_.set_fov(static_cast<float>(height()));
  ortho_camera_.set_camera_mode(AbstractCamera::CameraMode::OrthoMode);
  ortho_camera_.set_position(0.0f, 0.0f, 0.0f);

  ortho_camera_.UpdateWorld();
  ortho_camera_.UpdateProjection();

  QMatrix4x4 mvp =
      ortho_camera_.projection_matrix() * ortho_camera_.model_view_matrix();

  video_image_shader_prog_->bind();
  video_image_shader_prog_->setUniformValue(mvp_id_, mvp);
  video_image_shader_prog_->release();

  is_init_ = true;
}

void VideoImgViewer::resizeGL(int width, int height) {
  glViewport(0, 0, (GLsizei)width, (GLsizei)height);

  if (is_init_) {
    ortho_camera_.set_fov(static_cast<float>(height));
    ortho_camera_.set_near_plane_width(GLfloat(width));
    ortho_camera_.set_near_plane_height(GLfloat(height));
    ortho_camera_.UpdateProjection();

    QMatrix4x4 mvp =
        ortho_camera_.projection_matrix() * ortho_camera_.model_view_matrix();

    video_image_shader_prog_->bind();
    video_image_shader_prog_->setUniformValue(mvp_id_, mvp);
    video_image_shader_prog_->release();
  }
}

void VideoImgViewer::paintGL() {
  if (is_init_) {
    plane_.Render();
  }
}
