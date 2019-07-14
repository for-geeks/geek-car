/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <stdio.h>
#include <termios.h>
#include <iostream>
#include <memory>
#include <thread>

#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"

#include "modules/control/proto/chassis.pb.h"
#include "modules/control/proto/control.pb.h"

#include "cyber/common/log.h"

// gflags
DEFINE_double(throttle_inc_delta, 1.0,
              "throttle pedal command delta percentage.");
DEFINE_double(steer_inc_delta, 1.0, "steer delta percentage");

namespace {

using apollo::control::Chassis;
using apollo::control::Control_Command;
using apollo::cyber::CreateNode;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

const uint32_t KEYCODE_O = 0x4F;  // '0'

const uint32_t KEYCODE_UP1 = 0x57;  // 'w'
const uint32_t KEYCODE_UP2 = 0x77;  // 'w'
const uint32_t KEYCODE_DN1 = 0x53;  // 'S'
const uint32_t KEYCODE_DN2 = 0x73;  // 's'
const uint32_t KEYCODE_LF1 = 0x41;  // 'A'
const uint32_t KEYCODE_LF2 = 0x61;  // 'a'
const uint32_t KEYCODE_RT1 = 0x44;  // 'D'
const uint32_t KEYCODE_RT2 = 0x64;  // 'd'

const uint32_t KEYCODE_PKBK = 0x50;  // hand brake or parking brake

// set throttle, gear, and brake
const uint32_t KEYCODE_SETT1 = 0x54;  // 'T'
const uint32_t KEYCODE_SETT2 = 0x74;  // 't'
const uint32_t KEYCODE_ZERO = 0x30;   // '0'

const uint32_t KEYCODE_SETQ1 = 0x51;  // 'Q'
const uint32_t KEYCODE_SETQ2 = 0x71;  // 'q'

// emergency stop
const uint32_t KEYCODE_ESTOP = 0x45;  // 'E'

// help
const uint32_t KEYCODE_HELP = 0x68;   // 'h'
const uint32_t KEYCODE_HELP2 = 0x48;  // 'H'

class Teleop {
 public:
  Teleop() {
    ResetControlCommand();
    node_ = CreateNode("teleop");
  }
  static void PrintKeycode() {
    system("clear");
    printf("=====================    KEYBOARD MAP   ===================\n");
    printf("HELP:               [%c]     |\n", KEYCODE_HELP);
    printf("\n-----------------------------------------------------------\n");
    printf("Throttle/Speed up:  [%c]     |  Set Throttle:       [%c]+Num\n",
           KEYCODE_UP1, KEYCODE_SETT1);
    printf("Brake/Speed down:   [%c]     |  Set Brake:          []+Num\n",
           KEYCODE_DN1);
    printf("Steer LEFT:         [%c]     |  Steer RIGHT:        [%c]\n",
           KEYCODE_LF1, KEYCODE_RT1);
    printf("Parking Brake:     [%c]     |  Emergency Stop      [%c]\n",
           KEYCODE_PKBK, KEYCODE_ESTOP);
    printf("\n-----------------------------------------------------------\n");
    printf("Exit: Ctrl + C, then press enter to normal terminal\n");
    printf("===========================================================\n");
  }

  void KeyboardLoopThreadFunc() {
    char c = 0;
    int32_t level = 0;
    float throttle = 0;
    float steering = 0;
    struct termios cooked_;
    struct termios raw_;
    int32_t kfd_ = 0;
    Control_Command &control_command_ = control_command();

    // get the console in raw mode
    tcgetattr(kfd_, &cooked_);
    std::memcpy(&raw_, &cooked_, sizeof(struct termios));
    raw_.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw_.c_cc[VEOL] = 1;
    raw_.c_cc[VEOF] = 2;
    tcsetattr(kfd_, TCSANOW, &raw_);
    puts("Teleop:\nReading from keyboard now.");
    puts("---------------------------");
    puts("Use arrow keys to drive the car.");
    while (IsRunning()) {
      // get the next event from the keyboard
      if (read(kfd_, &c, 1) < 0) {
        perror("read():");
        exit(-1);
      }
      AINFO << "control command : "
            << control_command_.ShortDebugString().c_str();
      switch (c) {
        case KEYCODE_UP1:  // accelerate
        case KEYCODE_UP2:
          throttle = control_command_.throttle();
          throttle = GetCommand(throttle, (float)FLAGS_throttle_inc_delta);
          control_command_.set_throttle(throttle);

          AINFO << "Throttle = " << control_command_.throttle();
          break;
        case KEYCODE_DN1:  // decelerate
        case KEYCODE_DN2:
          throttle = control_command_.throttle();
          if (throttle > 1e-6) {
            throttle = GetCommand(throttle, (float)-FLAGS_throttle_inc_delta);
            control_command_.set_throttle(throttle);
          }
          AINFO << "Throttle = " << control_command_.throttle();
          break;
        case KEYCODE_LF1:  // left
        case KEYCODE_LF2:
          steering = control_command_.steer_angle();
          steering = GetCommand(steering, (float)FLAGS_steer_inc_delta);
          control_command_.set_steer_angle(steering);
          AINFO << "Steering Target = " << steering;
          break;
        case KEYCODE_RT1:  // right
        case KEYCODE_RT2:
          steering = control_command_.steer_angle();
          steering = GetCommand(steering, (float)-FLAGS_steer_inc_delta);
          control_command_.set_steer_angle(steering);
          AINFO << "Steering Target = " << steering;
          break;
        case KEYCODE_ESTOP:
          control_command_.set_throttle(0);
          AINFO << "Estop Brake : " << control_command_.throttle();
          break;
        case KEYCODE_SETT1:  // set throttle
        case KEYCODE_SETT2:
          // read keyboard again
          if (read(kfd_, &c, 1) < 0) {
            exit(-1);
          }
          level = c - KEYCODE_ZERO;
          AINFO << level;
          // control_command_.set_throttle(level * 10.0);
          // AINFO << "Throttle = " << control_command_.throttle();
          break;
        case KEYCODE_HELP:
        case KEYCODE_HELP2:
          PrintKeycode();
          break;
        default:
          // printf("%X\n", c);
          break;
      }
      AINFO << "control command after switch : "
            << control_command_.ShortDebugString().c_str();
    }  // keyboard_loop big while
    tcsetattr(kfd_, TCSANOW, &cooked_);
    AINFO << "keyboard_loop thread quited.";
    return;
  }  // end of keyboard loop thread

  Control_Command &control_command() { return control_command_; }

  float GetCommand(float val, float inc) {
    val += inc;
    if (val > 100.0) {
      val = 100.0;
    } else if (val < -100.0) {
      val = -100.0;
    }
    return val;
  }

  void Send() {
    control_command_writer_->Write(
        std::make_shared<Control_Command>(control_command_));
    ADEBUG << "Control Command send OK:" << control_command_.ShortDebugString();
  }

  void ResetControlCommand() {
    control_command_.Clear();
    control_command_.set_throttle(0.0);
    control_command_.set_steer_angle(0.0);
  }

  void OnChassis(const Chassis &chassis) { Send(); }

  int32_t Start() {
    if (is_running_) {
      AERROR << "Already running.";
      return -1;
    }
    is_running_ = true;
    chassis_reader_ = node_->CreateReader<Chassis>(
        "/chassis", [this](const std::shared_ptr<Chassis> &chassis) {
          OnChassis(*chassis);
        });
    control_command_writer_ = node_->CreateWriter<Control_Command>("/control");
    keyboard_thread_.reset(
        new std::thread([this] { KeyboardLoopThreadFunc(); }));
    if (keyboard_thread_ == nullptr) {
      AERROR << "Unable to create can client receiver thread.";
      return -1;
    }
    return 0;
  }

  void Stop() {
    if (is_running_) {
      is_running_ = false;
      if (keyboard_thread_ != nullptr && keyboard_thread_->joinable()) {
        keyboard_thread_->join();
        keyboard_thread_.reset();
        AINFO << "Teleop keyboard stopped [ok].";
      }
    }
  }

  bool IsRunning() const { return is_running_; }

 private:
  std::unique_ptr<std::thread> keyboard_thread_;
  std::shared_ptr<Reader<Chassis>> chassis_reader_;
  std::shared_ptr<Writer<Control_Command>> control_command_writer_;
  Control_Command control_command_;
  bool is_running_ = false;
  std::shared_ptr<apollo::cyber::Node> node_;
};

}  // namespace

int main(int32_t argc, char **argv) {
  apollo::cyber::Init(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;

  google::ParseCommandLineFlags(&argc, &argv, true);

  Teleop teleop;

  if (teleop.Start() != 0) {
    AERROR << "Teleop start failed.";
    return -1;
  }
  Teleop::PrintKeycode();
  apollo::cyber::WaitForShutdown();
  teleop.Stop();
  AINFO << "Teleop exit done.";
  return 0;
}
