// BSD 3-Clause License
//
// Copyright (c) 2026, National Institute of Advanced Industrial Science
// and Technology(AIST)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
// OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
// OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
//
/*!
 *  \file       ddynamic_reconfigure2.h
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

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
*  class param_range<T, NUMERIC>                                        *
************************************************************************/
template <class T, bool=!std::is_same_v<T, bool> && std::is_arithmetic_v<T> >
class param_range;

template <class T>
class param_range<T, true>
{
  public:
    using element_t = T;

  private:
    using range_t   = std::conditional_t<
                          std::is_integral_v<element_t>,
                          rcl_interfaces::msg::IntegerRange,
                          rcl_interfaces::msg::FloatingPointRange>;

  public:
    param_range(element_t from_value=std::numeric_limits<element_t>::min(),
                element_t to_value  =std::numeric_limits<element_t>::max(),
                element_t step=0)
    {
        _range.from_value = from_value;
        _range.to_value   = to_value;
        _range.step       = step;
    }

    rcl_interfaces::msg::ParameterDescriptor
    param_desc(const std::string& name, bool read_only=false) const
    {
        return param_desc(name, _range, read_only);
    }

    bool
    within(const element_t& value) const
    {
        return _range.from_value <= value && value <= _range.to_value;
    }

    element_t
    from_value() const
    {
        return _range.from_value;
    }

    element_t
    to_value() const
    {
        return _range.to_value;
    }

  private:
    static rcl_interfaces::msg::ParameterDescriptor
    param_desc(const std::string& name,
               const rcl_interfaces::msg::IntegerRange& range,
               bool read_only)
    {
        rcl_interfaces::msg::ParameterDescriptor        desc;
        desc.name = name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        if (range.from_value != std::numeric_limits<element_t>::min() ||
            range.to_value   != std::numeric_limits<element_t>::max())
            desc.integer_range.push_back(range);
        desc.read_only      = read_only;
        desc.dynamic_typing = false;

        return desc;
    }

    static rcl_interfaces::msg::ParameterDescriptor
    param_desc(const std::string& name,
               const rcl_interfaces::msg::FloatingPointRange& range,
               bool read_only)
    {
        rcl_interfaces::msg::ParameterDescriptor        desc;
        desc.name = name;
        desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        if (range.from_value != std::numeric_limits<element_t>::min() ||
            range.to_value   != std::numeric_limits<element_t>::max())
            desc.floating_point_range.push_back(range);
        desc.read_only      = read_only;
        desc.dynamic_typing = false;

        return desc;
    }

  private:
    range_t     _range;
};

template <class T>
class param_range<T, false>
{
  private:
    template <class T_> static T_       check_element(std::vector<T_>)  ;
    template <class T_> static T_       check_element(T_)               ;

  public:
    using element_t = decltype(check_element(std::declval<T>()));

  private:
    constexpr static uint8_t
        _type = (std::is_same_v<T, element_t> ?
                 (std::is_same_v<element_t, bool> ?
                  rcl_interfaces::msg::ParameterType::PARAMETER_BOOL :
                  rcl_interfaces::msg::ParameterType::PARAMETER_STRING) :
                 (std::is_same_v<element_t, bool> ?
                  rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY :
                  std::is_integral_v<element_t> ?
                  rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY :
                  std::is_floating_point_v<element_t> ?
                  rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY :
                  rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY));

  public:
    param_range(element_t=element_t(), element_t=element_t())           {}

    rcl_interfaces::msg::ParameterDescriptor
    param_desc(const std::string& name, bool read_only=false) const
    {
        rcl_interfaces::msg::ParameterDescriptor        desc;
        desc.name           = name;
        desc.type           = _type;
        desc.read_only      = read_only;
        desc.dynamic_typing = false;

        return desc;
    }

    bool        within(const T&)                const   { return true; }
    T           from_value()                    const   { return {}; }
    T           to_value()                      const   { return {}; }
};

/************************************************************************
*  class DDynamicReconfigure<NODE>                                      *
************************************************************************/
template <class NODE=rclcpp::Node>
class DDynamicReconfigure
{
  private:
    using param_cb_handle_p = std::shared_ptr<rclcpp::ParameterCallbackHandle>;
    using param_desc_t      = rcl_interfaces::msg::ParameterDescriptor;

  public:
                DDynamicReconfigure(const std::shared_ptr<NODE>& node)
                    :_node(node),
                     _param_event_handler(_node), _param_cb_handles()
                {
                }

    template <class T>
    void        registerVariable(const std::string& name, T* variable,
                                 const std::string& description="",
                                 const param_range<T>& range={})        ;
    template <class T>
    void        registerVariable(const std::string& name,
                                 const T& current_value,
                                 const std::function<void(const T&)>& cb,
                                 const std::string& description="",
                                 const param_range<T>& range={})        ;
    template <class T>
    void        registerEnumVariable(const std::string& name, T* variable,
                                     const std::string& description="",
                                     const std::map<std::string, T>&
                                                enum_dict={},
                                     const std::string& enum_description="");
    template <class T>
    void        registerEnumVariable(const std::string& name,
                                     const T& current_value,
                                     const std::function<void(const T&)>& cb,
                                     const std::string& description="",
                                     const std::map<std::string, T>&
                                                enum_dict={},
                                     const std::string& enum_description="");

  private:
    template <class T>
    void        registerParameter(const param_desc_t& desc,
                                  const T& current_value,
                                  const std::function<void(const T&)>& cb);

  private:
    std::shared_ptr<NODE>               _node;
    rclcpp::ParameterEventHandler       _param_event_handler;
    std::list<param_cb_handle_p>        _param_cb_handles;
};

template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerVariable(const std::string& name,
                                            T* variable,
                                            const std::string& description,
                                            const param_range<T>& range)
{
    registerVariable(name, *variable,
                     std::function<void(const T&)>([variable](const T& value)
                                                   { *variable = value; }),
                     description, range);
}

template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerVariable(
    const std::string& name, const T& current_value,
    const std::function<void(const T&)>& cb, const std::string& description,
    const param_range<T>& range)
{
    if (!range.within(current_value))
    {
        RCLCPP_WARN_STREAM(_node->get_logger(),
                           "Parameter[" << name
                           << "] is not registered because current value["
                           << current_value
                           << "] is out of range[" << range.from_value()
                           << ", " << range.to_value() << ']');
        return;
    }

    auto        desc = range.param_desc(name);
    desc.description = description;

    registerParameter(desc, current_value, cb);
}

template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerEnumVariable(
    const std::string& name, T* variable, const std::string& description,
    const std::map<std::string, T>& enum_dict,
    const std::string& enum_description)
{
    registerEnumVariable(name, *variable,
                         std::function<void(const T&)>(
                             [variable](const T& value){ *variable = value; }),
                         description, enum_dict, enum_description);
}

template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerEnumVariable(
    const std::string& name, const T& current_value,
    const std::function<void(const T&)>& cb, const std::string& description,
    const std::map<std::string, T>& enum_dict,
    const std::string& enum_description)
{
    if (enum_dict.empty())
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(),
                            "Trying to register an empty enum");
        throw;
    }

    auto        min = enum_dict.begin()->second;
    auto        max = min;
    for (const auto& val : enum_dict)
    {
        min = (min < val.second ? min : val.second);
        max = (max > val.second ? max : val.second);
    }

    auto        desc = param_range<T>(min, max).param_desc(name);
    desc.description = description;

    nlohmann::json      json;
    json["enum_description"] = enum_description;
    json["enum"] = nlohmann::json(enum_dict);
    desc.additional_constraints = json.dump();

    registerParameter(desc, current_value, cb);
}

template <class NODE> template <class T> void
DDynamicReconfigure<NODE>::registerParameter(
    const param_desc_t& desc, const T& current_value,
    const std::function<void(const T&)>& cb)
{
    try
    {
        _node->declare_parameter(desc.name, current_value, desc);
    }
    catch (const std::runtime_error& err)
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(), err.what());
        throw err;
    }

    _param_cb_handles.emplace_back(
        _param_event_handler.add_parameter_callback(
            desc.name,
            [cb, this](const rclcpp::Parameter& param)
            {
                cb(param.get_value<T>());
                RCLCPP_DEBUG_STREAM(_node->get_logger(),
                                    "Set parameter\"" << param.get_name()
                                    << "\" to " << param.get_value<T>());
            }));
}

/************************************************************************
*  utility functions                                                    *
************************************************************************/
template <class NODE_PTR, class T> T
declare_read_only_parameter(NODE_PTR&& node,
                            const std::string& name, const T& default_value)
{
    try
    {
        return node->declare_parameter(name, default_value,
                                       param_range<T>().param_desc(name,
                                                                   true));
    }
    catch (const std::runtime_error& err)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), err.what());
    }

    return default_value;
}

template <class NODE_PTR> std::string
declare_read_only_parameter(NODE_PTR&& node,
                            const std::string& name, const char* default_value)
{
    return declare_read_only_parameter(std::forward<NODE_PTR>(node),
                                       name, std::string(default_value));
}
}       // namespace ddynamic_reconfigure2
