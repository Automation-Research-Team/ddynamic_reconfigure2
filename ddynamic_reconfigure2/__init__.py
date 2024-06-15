# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Toshio Ueshiba
#
import json
from rclpy.node                    import Node
from rclpy.parameter_event_handler import ParameterEventHandler
from rcl_interfaces.msg            import (ParameterDescriptor, ParameterType,
                                           IntegerRange, FloatingPointRange)

#########################################################################
#  class DDynamicReconfigure                                            #
#########################################################################
class DDynamicReconfigure(object):
    def __init__(self, node):
        super().__init__()

        self._node = node
        self._param_event_handler = ParameterEventHandler(self._node)
        self._param_cb_handles = []

    def register_variable(self, param_name, param_type, current_value, cb,
                          description='',
                          min_value=None, max_value=None, step=0):
        self._register_parameter(self._create_desc(param_name, param_type,
                                                   description,
                                                   min_value, max_value, step),
                                 current_value, cb)

    def register_enum_variable(self, param_name, param_type, current_value, cb,
                               description, enum_dict, enum_description=''):
        desc = self._create_desc(param_name, param_type, description,
                                 min(enum_dict.values()),
                                 max(enum_dict.values()))
        desc.additional_constraints \
            = json.dumps({'enum_description': enum_description,
                          'enum': enum_dict})
        self._register_parameter(desc, current_value, cb)

    def _create_desc(self, param_name, param_type, description,
                     min_value, max_value, step=0):
        desc = ParameterDescriptor()
        desc.name           = param_name
        desc.type           = param_type
        desc.description    = description
        desc.read_only      = False
        desc.dynamic_typing = False
        if param_type == ParameterType.PARAMETER_INTEGER and \
           min_value is not None and max_value is not None:
            desc.integer_range.append(IntegerRange(from_value=min_value,
                                                   to_value=max_value,
                                                   step=step))
        elif param_type == ParameterType.PARAMETER_DOUBLE and \
             min_value is not None and max_value is not None:
            desc.integer_range.append(FloatingPointRange(from_value=min_value,
                                                         to_value=max_value,
                                                         step=float(step)))
        return desc

    def _register_parameter(self, desc, current_value, cb):
        self._node.declare_parameter(desc.name, current_value, desc)
        self._param_cb_handles.append(
            self._param_event_handler.add_parameter_callback(
                desc.name, self.get_name(), lambda param: cb(param.value)))
