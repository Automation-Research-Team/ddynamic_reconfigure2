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
 *  \file	ddynamic_reconfigure2.h
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ddynamic_reconfigure2
{
/************************************************************************
*  class param_range<T>							*
************************************************************************/
template <class T>
struct param_range;

template <>
struct param_range<bool>
{
    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;

	return desc;
    }
};

template <>
struct param_range<int64_t>
{
    param_range(int64_t from_value_=std::numeric_limits<int64_t>::min(),
		int64_t to_value_  =std::numeric_limits<int64_t>::max(),
		uint64_t step_=1)
	:from_value(from_value_), to_value(to_value_), step(step_)
    {
    }

    rcl_interfaces::msg::IntegerRange
    integer_range() const
    {
	rcl_interfaces::msg::IntegerRange	rng;
	rng.from_value = from_value;
	rng.to_value   = to_value;
	rng.step       = step;

	return rng;
    }

    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
	desc.integer_range.push_back(integer_range());

	return desc;
    }

    int64_t	from_value;
    int64_t	to_value;
    uint64_t	step;
};

template <>
struct param_range<double>
{
    param_range(double from_value_=std::numeric_limits<double>::min(),
		double to_value_  =std::numeric_limits<double>::max(),
		double step_=0)
	:from_value(from_value_), to_value(to_value_), step(step_)
    {
    }

    rcl_interfaces::msg::FloatingPointRange
    floating_point_range() const
    {
	rcl_interfaces::msg::FloatingPointRange	rng;
	rng.from_value = from_value;
	rng.to_value   = to_value;
	rng.step       = step;

	return rng;
    }

    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
	desc.floating_point_range.push_back(floating_point_range());

	return desc;
    }

    double	from_value;
    double	to_value;
    double	step;
};

template <>
struct param_range<std::string>
{
    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;

	return desc;
    }
};

template <>
struct param_range<std::vector<bool> >
{
    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;

	return desc;
    }
};

template <>
struct param_range<std::vector<int64_t> >
{
    param_range(const std::vector<param_range<int64_t> >& ranges)
	:_ranges(ranges)
    {
    }

    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
	for (const auto& range : _ranges)
	    desc.integer_range.push_back(range.integer_range());

	return desc;
    }

  private:
    const std::vector<param_range<int64_t> >	_ranges;
};

template <>
struct param_range<std::vector<double> >
{
    param_range(const std::vector<param_range<double> >& ranges)
	:_ranges(ranges)
    {
    }

    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
	for (const auto& range : _ranges)
	    desc.floating_point_range.push_back(range.floating_point_range());

	return desc;
    }

  private:
    const std::vector<param_range<double> >	_ranges;
};

template <>
struct param_range<std::vector<std::string> >
{
    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;

	return desc;
    }
};

/************************************************************************
*  class DDynamicReconfigure						*
************************************************************************/
class DDynamicReconfigure
{
  private:
    using cb_handle_wp	  = std::weak_ptr<rclcpp::ParameterCallbackHandle>;

  public:
		DDynamicReconfigure(rclcpp::Node::SharedPtr node)
		    :_node(node), _param_handler(_node), _param_cb_handles()
		{
		}

    template <class T>
    void	registerVariable(const std::string& name, T* variable,
				 const std::string& description="",
				 const param_range<T>& range={},
				 const std::string& group="Default")	;
    template <class T>
    void	registerVariable(const std::string& name,
				 const T& current_value,
				 const std::function<void(const T&)>& cb,
				 const std::string& description="",
				 const param_range<T>& range={},
				 const std::string& group="Default")	;

  private:
    rclcpp::Node::SharedPtr		_node;
    rclcpp::ParameterEventHandler	_param_handler;
    std::list<cb_handle_wp>		_param_cb_handles;
};

}	// namespace ddynamic_reconfigure2
