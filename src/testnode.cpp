// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
/*!
 *  \file	testnode.cpp
 */
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.h>

namespace ddynamic_reconfigure2
{
class TestNode : public rclcpp::Node
{
  public:
    TestNode(const std::string& node_name,
	     const rclcpp::NodeOptions& options=rclcpp::NodeOptions())	;

  private:
    DDynamicReconfigure	_ddr;
    int64_t		_param_i64;
    double		_param_d;
};

TestNode::TestNode(const std::string& node_name,
		   const rclcpp::NodeOptions& options)
    :rclcpp::Node(node_name, options),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _param_i64(3), _param_d(0.5)
{
    _ddr.registerVariable("param_i64", &_param_i64,
			  "parameter of int64_t type", {-4, 10, 2});
    _ddr.registerVariable("param_d", &_param_d,
			  "parameter of double type", {-1.0, 2.0, 0.1});
}
}	// ddynamic_reconfigure2

int
main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ddynamic_reconfigure2::TestNode>("testnode");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
