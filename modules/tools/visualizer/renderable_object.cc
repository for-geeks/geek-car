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

#include "modules/tools/visualizer/renderable_object.h"

#include <iostream>

std::shared_ptr<QOpenGLShaderProgram> RenderableObject::NullRenderableObj;

std::shared_ptr<QOpenGLShaderProgram> RenderableObject::CreateShaderProgram(
    const char *vertexShaderSource, const char *fragmentShaderSource) {
  std::shared_ptr<QOpenGLShaderProgram> shaderProg(new QOpenGLShaderProgram());
  if (shaderProg != nullptr) {
    // static const char *vertexShaderSource =
    //     "#version 130\n"
    //     "in vec2 vertPos;\n"
    //     "in vec2 texCoord;\n"
    //     "out vec2 TexCoord;\n"
    //     "void main() {\n"
    //     "   gl_Position = vec4(vertPos.x, vertPos.y, 0.0, 1.0);\n"
    //     "   TexCoord = vec2(texCoord.x, 1.0 - texCoord.y);\n"
    //     "}\n";


    shaderProg->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                        vertexShaderSource);

    // static const char *fragmentShaderSource =
    // "#version 130\n"
    // "in mediump vec2 TexCoord;\n"
    // "out mediump vec4 FragColor;\n"
    // "uniform sampler2D texture;\n"
    // "void main() {\n"
    // "   FragColor = texture2D(texture, TexCoord);\n"
    // "}\n";

    shaderProg->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                        fragmentShaderSource);
    if (!shaderProg->link()) {
      std::cerr << "----cannot link shader programm, log:("
                << shaderProg->log().toStdString() << ")\n";

      shaderProg.reset();
    }
  }
  return shaderProg;
}

RenderableObject::RenderableObject(
    int vertexCount, int vertexElementCount,
    const std::shared_ptr<QOpenGLShaderProgram>& shaderProgram)
    : QOpenGLFunctions(),
      is_init_(false),
      is_renderable_(true),
      vertex_count_(vertexCount),
      vertex_element_count_(vertexElementCount),
      shader_program_(shaderProgram),
      vao_(),
      vbo_(QOpenGLBuffer::VertexBuffer) {}

RenderableObject::~RenderableObject() { Destroy(); }

void RenderableObject::Destroy(void) {
  if (is_init_) {
    is_renderable_ = false;
    is_init_ = false;
    shader_program_.reset();
    vbo_.destroy();
    vao_.destroy();
  }
}

bool RenderableObject::Init(
    std::shared_ptr<QOpenGLShaderProgram>& shaderProgram) {
  if (is_init_) {
    return true;
  }

  if (vertex_count() < 1) {
    return false;
  }
  if (vertex_element_count() < 1) {
    return false;
  }

  if (shaderProgram != nullptr) {
    shader_program_ = shaderProgram;
  }

  if (shader_program_ == nullptr) {
    return false;
  }

  initializeOpenGLFunctions();

  if (!vao_.create()) {
    return false;
  }
  vao_.bind();

  if (!vbo_.create()) {
    vao_.destroy();
    return false;
  }
  vbo_.setUsagePattern(QOpenGLBuffer::StaticDraw);
  vbo_.bind();

  vbo_.allocate(VertexBufferSize());
  GLfloat* pBuffer = static_cast<GLfloat*>(vbo_.map(QOpenGLBuffer::WriteOnly));
  bool ret = FillVertexBuffer(pBuffer);
  vbo_.unmap();

  if (!ret) {
    return ret;
  }

  SetupAllAttrPointer();

  vbo_.release();
  vao_.release();
  is_init_ = true;

  return true;
}

void RenderableObject::Render(const QMatrix4x4* mvp) {
  if (is_init_) {
    if (is_renderable()) {
      shader_program_->bind();
      if (mvp) shader_program_->setUniformValue("mvp", *mvp);
      SetupExtraUniforms();
      vao_.bind();
      Draw();
      vao_.release();
      shader_program_->release();
    }
  } else {
    std::cerr << "Please initialize the object" << std::endl;
  }
}
