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
template <class T> static std::ostream&
operator <<(std::ostream& out, const std::vector<T>& v)
{
    for (const auto& val : v)
	out << ' ' << val;
    return out;
}

/************************************************************************
*  class TestNode							*
************************************************************************/
class TestNode : public rclcpp::Node
{
  public:
    TestNode(const std::string& node_name,
	     const rclcpp::NodeOptions& options=rclcpp::NodeOptions())	;

  private:
    void	timer_cb()						;

  private:
    rclcpp::TimerBase::SharedPtr	_timer;
    DDynamicReconfigure			_ddr;
    bool				_param_b;
    int64_t				_param_i64;
    double				_param_d;
    std::string				_param_s;
    std::vector<bool>			_params_b;
    std::vector<int64_t>		_params_i64;
    std::vector<double>			_params_d;
    std::vector<std::string>		_params_s;
    double				_enum_param_d;
    std::string				_enum_param_s;
};

TestNode::TestNode(const std::string& node_name,
		   const rclcpp::NodeOptions& options)
    :rclcpp::Node(node_name, options),
     _timer(nullptr),
     _ddr(rclcpp::Node::SharedPtr(this)),
     _param_b(true), _param_i64(4), _param_d(0.5), _param_s("str0"),
     _params_b({false, true}), _params_i64({2, 11}), _params_d({0.1, 0.4}),
     _params_s({"s0", "s1"}), _enum_param_d(2.1), _enum_param_s("Two")
{
    using	namespace std::chrono_literals;

    _ddr.registerVariable("param_b", &_param_b, "parameter of bool type");
    _ddr.registerVariable("param_i64", &_param_i64,
			  "parameter of int64_t type", {-4, 10, 2}, "numeric");
    _ddr.registerVariable("param_d", &_param_d,
			  "parameter of double type", -1.0, 2.0, "numeric");
    _ddr.registerVariable("param_s", &_param_s, "parameter of string type");
    _ddr.registerVariable("params_b", &_params_b,
			  "parameter array of bool type");
    _ddr.registerVariable("params_i64", &_params_i64,
			  "parameter array of int64_t type");
    _ddr.registerVariable("params_d", &_params_d,
			  "parameter array of double type");
    _ddr.registerVariable("params_s", &_params_s,
			  "parameter array of string type");
    _ddr.registerEnumVariable("enum_param_d", &_enum_param_d,
			      "enum parameter of double type",
			      {{"low", 1.0}, {"middle", 2.1}, {"high", 3.2}},
			      "low/middle/high", "numeric");
    _ddr.registerEnumVariable("enum_param_s", &_enum_param_s,
			      "enum parameter of string type",
			      {{"one", "One"}, {"two", "Two"},
			       {"three", "Three"}},
			      "one/two/three");

    _timer = create_wall_timer(1000ms, std::bind(&TestNode::timer_cb, this));
}

void
TestNode::timer_cb()
{
    RCLCPP_INFO_STREAM(get_logger(), "param_b[" << _param_b
		       << "] param_i64[" << _param_i64
		       << "] param_d[" << _param_d
		       << "] param_s[" << _param_s
		       << "] params_b[" << _params_b
		       << "] params_i64[" << _params_i64
		       << "] params_d[" << _params_d
		       << "] params_s[" << _params_s
		       << "] enum_param_d[" << _enum_param_d
		       << "] enum_param_s[" << _enum_param_s
		       << ']');
}
}	// ddynamic_reconfigure2

/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ddynamic_reconfigure2::TestNode>("testnode");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
