// Copyright 2026 enabl Technologies GmbH
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// A device that starts with a wrong default on every axis and sends its real state with its
// first report (the Saitek Side Panel Control Deck: the kernel starts all axes at 0 of 0..255,
// which SDL reports as -32768) must not leave axes stale in the published Joy message.
//
// On that device SDL updated its internal state for most axes without sending an
// SDL_JOYAXISMOTION event for them, so the event-driven node kept the initial -32768 (+1.0).
// This test reproduces the missing events deterministically by disabling SDL's axis-motion
// event type: SDL still tracks the axis state, but the node is never told about it.

#include <SDL.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "joy/joy.hpp"

namespace
{

constexpr int kNumAxes = 6;
constexpr int16_t kMinimum = -32768;
constexpr char kDeviceName[] = "Joy axis resync test device";

// Spin until the latest Joy message satisfies the predicate, or the timeout elapses.
template<typename Pred>
bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor, const std::vector<float> & axes,
  Pred pred, std::chrono::milliseconds timeout)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    executor.spin_some(std::chrono::milliseconds(20));
    if (pred(axes)) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(JoyAxisResync, AxesFollowSdlStateEvenWhenSdlSendsNoEventForThem)
{
  ASSERT_EQ(0, SDL_Init(SDL_INIT_JOYSTICK));

  SDL_VirtualJoystickDesc desc;
  SDL_zero(desc);
  desc.version = SDL_VIRTUAL_JOYSTICK_DESC_VERSION;
  desc.type = SDL_JOYSTICK_TYPE_UNKNOWN;
  desc.naxes = kNumAxes;
  desc.name = kDeviceName;
  const int index = SDL_JoystickAttachVirtualEx(&desc);
  ASSERT_GE(index, 0) << SDL_GetError();

  SDL_Joystick * own = SDL_JoystickOpen(index);
  ASSERT_NE(nullptr, own) << SDL_GetError();

  // The device starts with every axis at the minimum, and SDL registers that as the axis's
  // initial value.
  for (int a = 0; a < kNumAxes; ++a) {
    SDL_JoystickSetVirtualAxis(own, a, kMinimum);
  }
  SDL_JoystickUpdate();

  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("device_name", std::string(kDeviceName));
    options.append_parameter_override("autorepeat_rate", 100.0);
    auto joy_node = std::make_shared<joy::Joy>(options);

    auto sub_node = std::make_shared<rclcpp::Node>("joy_axis_resync_test_sub");
    std::vector<float> axes;
    auto sub = sub_node->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::QoS(10), [&axes](sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        axes = msg->axes;
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sub_node);

    // At open the node reads the initial state: all axes at the minimum (+1.0 in ROS).
    ASSERT_TRUE(
      spinUntil(
        executor, axes, [](const std::vector<float> & a) {
          return a.size() == kNumAxes &&
                 std::all_of(a.begin(), a.end(), [](float v) {return v == 1.0F;});
        },
        std::chrono::seconds(5)));

    // The device now reports its real state, and SDL does not announce it with axis events.
    SDL_EventState(SDL_JOYAXISMOTION, SDL_IGNORE);
    for (int a = 0; a < kNumAxes; ++a) {
      SDL_JoystickSetVirtualAxis(own, a, 0);
    }

    // Every axis must follow SDL's state, whether or not an axis event was delivered.
    EXPECT_TRUE(
      spinUntil(
        executor, axes, [](const std::vector<float> & a) {
          return a.size() == kNumAxes &&
                 std::all_of(a.begin(), a.end(), [](float v) {return v == 0.0F;});
        },
        std::chrono::seconds(3)))
      << "stale axes in the published Joy message";
    SDL_EventState(SDL_JOYAXISMOTION, SDL_ENABLE);
  }
  rclcpp::shutdown();

  SDL_JoystickClose(own);
  SDL_JoystickDetachVirtual(index);
  SDL_Quit();
}
