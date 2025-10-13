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
import rclpy, time
from rclpy.parameter        import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

#########################################################################
#  class ParametrerClient                                               #
#########################################################################
class ParameterClient(AsyncParameterClient):
    def __init__(self, node, remote_node_name,
                 qos_profile=rclpy.qos.qos_profile_services_default,
                 callback_group=None):
        super().__init__(node, remote_node_name, qos_profile=qos_profile,
                         callback_group=callback_group)

    def get_parameters_sync(self, names):
        future = self.get_parameters(names)
        while not future.done():
            time.sleep(0.1)
        return [parameter_value_to_python(value)
                for value in future.result().values]

    def set_parameters_sync(self, param_tuples):
        parameters = [Parameter(param_tuple[0],
                                value=param_tuple[1]).to_parameter_msg()
                      for param_tuple in param_tuples]
        future = self.set_parameters(parameters)
        while not future.done():
            time.sleep(0.1)
        return future.result().results
