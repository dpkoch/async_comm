/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2018 Daniel Koch.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file message_handler_ros.h
 * @author Daniel Koch <danielpkoch@gmail.com>
 */

#ifndef ASYNC_COMM_MESSAGE_HANDLER_ROS_H
#define ASYNC_COMM_MESSAGE_HANDLER_ROS_H

#include <async_comm/message_handler.h>

#include <ros/ros.h>

namespace async_comm
{
namespace util
{
/**
 * @class MessageHandlerROS
 * @brief Message handler implementation for ROS environments
 *
 * This is a convenience message handler implementation for ROS-based projects.
 * The implementation simply forwards messages to the appropriate rosconsole
 * loggers.
 */
class MessageHandlerROS : public MessageHandler
{
public:
  inline void debug(const std::string &message) override { ROS_DEBUG("[async_comm]: %s", message.c_str()); }
  inline void info(const std::string &message) override { ROS_INFO("[async_comm]: %s", message.c_str()); }
  inline void warn(const std::string &message) override { ROS_WARN("[async_comm]: %s", message.c_str()); }
  inline void error(const std::string &message) override { ROS_ERROR("[async_comm]: %s", message.c_str()); }
  inline void fatal(const std::string &message) override { ROS_FATAL("[async_comm]: %s", message.c_str()); }
};

}  // namespace util
}  // namespace async_comm

#endif  // ASYNC_COMM_MESSAGE_HANDLER_ROS_H