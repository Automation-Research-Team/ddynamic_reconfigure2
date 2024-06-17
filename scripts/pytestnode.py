#!/usr/bin/env python3
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
import rclpy
from rclpy.node            import Node
from rcl_interfaces.msg    import ParameterType
from ddynamic_reconfigure2 import DDynamicReconfigure

#########################################################################
#  class TestNode                                                       #
#########################################################################
class TestNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self._param_b      = True
        self._param_i64    = 4
        self._param_d      = 0.5
        self._param_s      = 'str0'
        self._params_b     = [False, True]
        self._params_i64   = [2, 11]
        self._params_d     = [0.1, 0.4]
        self._params_s     = ['s0', 's1']
        self._enum_param_d = 2.1
        self._enum_param_s = 'Two'

        self._ddr = DDynamicReconfigure(self)
        self._ddr.register_variable('param_b',
                                    ParameterType.PARAMETER_BOOL,
                                    self._param_b,
                                    lambda x: setattr(self, '_param_b', x),
                                    'parameter of bool type')
        self._ddr.register_variable('numeric.param_i64',
                                    ParameterType.PARAMETER_INTEGER,
                                    self._param_i64,
                                    lambda x: setattr(self, '_param_i64', x),
                                    'parameter of int64_t type',
                                    -4, 10, 2)
        self._ddr.register_variable('numeric.param_d',
                                    ParameterType.PARAMETER_DOUBLE,
                                    self._param_d,
                                    lambda x: setattr(self, '_param_d', x),
                                    'parameter of double type',
                                    -1.0, 2.0)
        self._ddr.register_variable('string.param_s',
                                    ParameterType.PARAMETER_STRING,
                                    self._param_s,
                                    lambda x: setattr(self, '_param_s', x),
                                    'parameter of string type')
        self._ddr.register_variable('params_b',
                                    ParameterType.PARAMETER_BOOL_ARRAY,
                                    self._params_b,
                                    lambda x: setattr(self, '_params_b', x),
                                    'parameter of bool array type')
        self._ddr.register_variable('numeric.params_i64',
                                    ParameterType.PARAMETER_INTEGER_ARRAY,
                                    self._params_i64,
                                    lambda x: setattr(self, '_params_i64', x),
                                    'parameter of int64_t array type')
        self._ddr.register_variable('numeric.params_d',
                                    ParameterType.PARAMETER_DOUBLE_ARRAY,
                                    self._params_d,
                                    lambda x: setattr(self, '_params_d', x),
                                    'parameter of double array type')
        self._ddr.register_variable('string.params_s',
                                    ParameterType.PARAMETER_STRING_ARRAY,
                                    self._params_s,
                                    lambda x: setattr(self, '_params_s', x),
                                    'parameter of string array type')
        self._ddr.register_enum_variable('numeric.enum_param_d',
                                         ParameterType.PARAMETER_DOUBLE,
                                         self._enum_param_d,
                                         lambda x: setattr(self,
                                                           '_enum_param_d', x),
                                         'enum parameter of double type',
                                         {'low': 1.0,
                                          'middle': 2.1, 'high': 3.2},
                                         'low/middle/high')
        self._ddr.register_enum_variable('string.enum_param_s',
                                         ParameterType.PARAMETER_STRING,
                                         self._enum_param_s,
                                         lambda x: setattr(self,
                                                           '_enum_param_s', x),
                                         'enum parameter of string type',
                                         {'one': 'One',
                                          'two': 'Two', 'three': 'Three'},
                                         'one/two/three')

        self._timer = self.create_timer(1.0, self._timer_cb)

    def _timer_cb(self):
        self.get_logger().info('param_b[{0}] param_i64[{1}] param_d[{2}] param_s[{3}] params_b[{4}] params_i64[{5}] params_d[{6}] params_s[{7}] enum_param_d[{8}] enum_param_s[{9}]'
                               .format(self._param_b, self._param_i64,
                                       self._param_d, self._param_s,
                                       self._params_b, self._params_i64,
                                       self._params_d, self._params_s,
                                       self._enum_param_d, self._enum_param_s))


def main(args):
    rclpy.init(args=args)

    node = TestNode('pytestnode')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(None)
