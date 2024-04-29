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
*  class param_range<T, N>						*
************************************************************************/
namespace detail
{
  template <class T> static T	check_element(std::vector<T>)		;
  template <class T> static T	check_element(T)			;

  template <class T>
  using element_t = decltype(check_element(std::declval<T>()));
  template <class T>
  using is_vec = std::negation<std::is_same<T, element_t<T> > >;
}

template <class T, bool=std::is_same<bool, T>::value ||
			detail::is_vec<T>::value ||
			!std::is_arithmetic<T>::value>
class param_range;

template <class T>
class param_range<T, false>
{
  public:
    using element_t = T;

  private:
    using range_t   = std::conditional_t<
			  std::is_same<element_t, int64_t>::value,
			  rcl_interfaces::msg::IntegerRange,
			  rcl_interfaces::msg::FloatingPointRange>;

  public:
    param_range(element_t from_value=std::numeric_limits<element_t>::min(),
		element_t to_value  =std::numeric_limits<element_t>::max(),
		element_t step=0)
	:_from_value(from_value), _to_value(to_value), _step(step)
    {
    }

    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	range_t	range;
	range.from_value = _from_value;
	range.to_value   = _to_value;
	range.step       = _step;

	return set_range(range);
    }

  private:
    rcl_interfaces::msg::ParameterDescriptor
    set_range(const rcl_interfaces::msg::IntegerRange& range) const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
	desc.integer_range.push_back(range);

	return desc;
    }

    rcl_interfaces::msg::ParameterDescriptor
    set_range(const rcl_interfaces::msg::FloatingPointRange& range) const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
	desc.floating_point_range.push_back(range);

	return desc;
    }

  private:
    element_t	_from_value;
    element_t	_to_value;
    element_t	_step;
};

template <class T>
class param_range<T, true>
{
  public:
    using element_t = detail::element_t<T>;

  private:
    constexpr static uint8_t
	_type = (detail::is_vec<T>::value ?
		 (std::is_same<element_t, bool>::value ?
		  rcl_interfaces::msg::ParameterType::PARAMETER_BOOL :
		  rcl_interfaces::msg::ParameterType::PARAMETER_STRING) :
		 (std::is_same<element_t, bool>::value ?
		  rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY :
		  std::is_same<element_t, int64_t>::value ?
		  rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY :
		  std::is_same<element_t, double>::value ?
		  rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY :
		  rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY));

  public:
    rcl_interfaces::msg::ParameterDescriptor
    param_desc() const
    {
	rcl_interfaces::msg::ParameterDescriptor	desc;
	desc.type = _type;

	return desc;
    }
};

/************************************************************************
*  class DDynamicReconfigure						*
************************************************************************/
class DDynamicReconfigure
{
  private:
    using param_cb_handle_p = std::shared_ptr<rclcpp::ParameterCallbackHandle>;

  public:
		DDynamicReconfigure(rclcpp::Node::SharedPtr node)
		    :_node(node),
		     _param_event_handler(_node), _param_cb_handles()
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
    rclcpp::ParameterEventHandler	_param_event_handler;
    std::list<param_cb_handle_p>	_param_cb_handles;
};

}	// namespace ddynamic_reconfigure2
