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
template void	DDynamicReconfigure::registerParameter(
		    const param_desc_t& desc, const bool& current_value,
		    const std::function<void(const bool&)>& cb)		;
template void	DDynamicReconfigure::registerParameter(
		    const param_desc_t& desc, const int64_t& current_value,
		    const std::function<void(const int64_t&)>& cb)	;
template void	DDynamicReconfigure::registerParameter(
		    const param_desc_t& desc, const double& current_value,
		    const std::function<void(const double&)>& cb)	;
template void	DDynamicReconfigure::registerParameter(
		    const param_desc_t& desc,
		    const std::string& current_value,
		    const std::function<void(const std::string&)>& cb)	;
template void	DDynamicReconfigure::registerParameter(
		    const param_desc_t& desc,
		    const std::vector<bool>& current_value,
		    const std::function<void(const std::vector<bool>&)>& cb);
template void	DDynamicReconfigure::registerParameter(
		    const param_desc_t& desc,
		    const std::vector<int64_t>& current_value,
		    const std::function<
				void(const std::vector<int64_t>&)>& cb);
template void	DDynamicReconfigure::registerParameter(
		    const param_desc_t& desc,
		    const std::vector<double>& current_value,
		    const std::function<void(const std::vector<double>&)>& cb);
template void	DDynamicReconfigure::registerParameter(
		    const param_desc_t& desc,
		    const std::vector<std::string>& current_value,
		    const std::function<
				void(const std::vector<std::string>&)>& cb);
}	// namespace ddynamic_reconfigure2
