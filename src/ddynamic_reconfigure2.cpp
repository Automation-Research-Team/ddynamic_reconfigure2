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
    using param_t	= rclcpp::Parameter;

    auto	desc = range.param_desc();
    desc.name		= name;
    desc.description	= description;
    desc.read_only	= false;
    desc.dynamic_typing = false;
    _node->declare_parameter(name, current_value, desc);

    _param_cb_handles.emplace_back(
	_param_handler.add_parameter_callback(name,
					      [&cb](const param_t& param)
					      { cb(param.get_value<T>()); }));
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
    const std::string& description, const param_range<int64_t>& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, double* variable,
    const std::string& description, const param_range<double>& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, std::string* variable,
    const std::string& description, const param_range<std::string>& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, const bool& current_value,
    const std::function<void(const bool& value)>& cb,
    const std::string& description, const param_range<bool>& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, const int64_t& current_value,
    const std::function<void(const int64_t& value)>& cb,
    const std::string& description, const param_range<int64_t>& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, const double& current_value,
    const std::function<void(const double& value)>& cb,
    const std::string& description, const param_range<double>& range,
    const std::string& group);

template void
DDynamicReconfigure::registerVariable(
    const std::string& name, const std::string& current_value,
    const std::function<void(const std::string& value)>& cb,
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
