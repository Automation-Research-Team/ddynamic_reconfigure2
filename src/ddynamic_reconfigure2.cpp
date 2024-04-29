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
 *  \file	ddynamic_reconfigure2.cpp
 */
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.h>

namespace ddynamic_reconfigure2
{
/************************************************************************
*  static functions (for debugging)					*
************************************************************************/
static std::ostream&
operator <<(std::ostream& out,
	    const rcl_interfaces::msg::ParameterDescriptor& desc)
{
    out << "name: " << desc.name
	<< "\ntype: " << int(desc.type)
	<< "\nread_only: " << desc.read_only
	<< "\nfloating_point_range:";
    for (const auto& range : desc.floating_point_range)
	out << " [" << range.from_value << "," << range.to_value
	    << ':' << range.step << ']';
    out << "\ninteger_range:";
    for (const auto& range : desc.integer_range)
	out << " [" << range.from_value << "," << range.to_value
	    << ':' << range.step << ']';
    return out;
}

/************************************************************************
*  class DDynamicReconfigure						*
************************************************************************/
template <class T> void
DDynamicReconfigure::registerVariable(const std::string& name, T* variable,
				      const std::string& description,
				      const param_range<T>& range,
				      const std::string& group)
{
    registerVariable(name, *variable,
		     std::function<void(const T&)>([variable](const T& value)
						   { *variable = value; }),
		     description, range, group);
}

template <class T> void
DDynamicReconfigure::registerVariable(const std::string& name,
				      const T& current_value,
				      const std::function<void(const T&)>& cb,
				      const std::string& description,
				      const param_range<T>& range,
				      const std::string& group)
{
    auto	desc = range.param_desc();
    desc.name		= name;
    desc.description	= description;
    desc.read_only	= false;
    desc.dynamic_typing = false;
  //std::cerr << desc << std::endl;
    _node->declare_parameter(name, current_value, desc);

    _param_cb_handles.emplace_back(
	_param_event_handler.add_parameter_callback(
	    name,
	    [cb](const rclcpp::Parameter& param){cb(param.get_value<T>());}));
}

template <class T> void
DDynamicReconfigure::registerVariable(const std::string& name, T* variable,
				      const std::string& description,
				      T min, T max, const std::string& group)
{
    registerVariable(name, variable, description,
		     param_range<T>(min, max, 0), group);
}

template <class T> void
DDynamicReconfigure::registerVariable(const std::string& name,
				      const T& current_value,
				      const std::function<void(const T&)>& cb,
				      const std::string& description,
				      T min, T max, const std::string& group)
{
    registerVariable(name, current_value, description,
		     param_range<T>(min, max, 0), group);
}
    
// Instantiations
template void
DDynamicReconfigure::registerVariable(
    const std::string& name, bool* variable,
    const std::string& description, const param_range<bool>& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, int64_t* variable,
    const std::string& description, int64_t min, int64_t max,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, double* variable,
    const std::string& description, double min, double max,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, std::string* variable,
    const std::string& description, const param_range<std::string>& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, std::vector<bool>* variable,
    const std::string& description,
    const param_range<std::vector<bool> >& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, std::vector<int64_t>* variable,
    const std::string& description,
    const param_range<std::vector<int64_t> >& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, std::vector<double>* variable,
    const std::string& description,
    const param_range<std::vector<double> >& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, std::vector<std::string>* variable,
    const std::string& description,
    const param_range<std::vector<std::string> >& range,
    const std::string& group);

}	// namespace ddynamic_reconfigure2
