/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "husky_base/husky_hardware.hpp"
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

const int DEFAULT_UPDATE_RATE = 100;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor,
    manager_node_name);

  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // When the MutliThreadedExecutor issues are fixed (ros2/rclcpp#1168), this loop should be
  // converted back to a timer.
  std::thread cm_thread([cm]() {
      // load controller_manager update time parameter
      int update_rate = DEFAULT_UPDATE_RATE;
      if (!cm->get_parameter("update_rate", update_rate)) 
      {
        RCLCPP_WARN(cm->get_logger(), "'update_rate' parameter not set, using default value.");
      }
      RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", update_rate);

      while (rclcpp::ok()) 
      {
        std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();
        cm->read();
        cm->update();
        cm->write();
        std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
        std::this_thread::sleep_for(
          std::max(
            std::chrono::nanoseconds(0),
            std::chrono::nanoseconds(1000000000 / update_rate) -
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin)));
      }
    });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return 0;
}