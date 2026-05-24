#  BSD 3-Clause License
#
#  Copyright (c) 2026, National Institute of Advanced Industrial Science
#  and Technology(AIST)
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
#  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
#  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
""" Defines python implementation of ParameterClient.
"""
import threading
import rclpy
from rclpy.parameter        import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

from rclpy.node             import Node
from rclpy.qos              import QoSProfile, qos_profile_services_default
from rclpy.callback_groups  import CallbackGroup
from rcl_interfaces.msg     import Parameter as ParameterMsg
from typing                 import Any, List, Optional, Sequence, Union

#************************************************************************
#  class ParametrerClient                                               *
#************************************************************************
class ParameterClient(AsyncParameterClient):
    """ Extention of AsyncParameterClient with synchronus get/set methods.
    """
    def __init__(self, node: Node, remote_node_name: str,
                 qos_profile: QoSProfile=qos_profile_services_default,
                 callback_group: Optional[CallbackGroup]=None):
        """
        An ParameterClient is a subclass
        of rclpy.parameter_client.AsyncParameterClinet which adds two
        methods for getting/setting parameters of remote node
        in a synchronous manner.

        Args:
          node: Node used to create clients that will interact
            with the remote node.
          remote_node_name: Name of remote node for which the parameters
            will be managed.
          qos_profile: Profile of QoS.
          callback_group: Callback group to add the parameter client to.
            If `None`, then the node's default callback group is used.
        """
        super().__init__(node, remote_node_name, qos_profile=qos_profile,
                         callback_group=callback_group)

    def get_parameters_sync(self, names: List[str], *,
                            timeout_sec: Optional[float]=None):
        """ Get parameters with given names.

        Args:
          names: List of parameter names to get.
          timeout_sec: Timeout time waiting for remote parameters
            being obtained. Seconds to wait. Wait forever, if `None`.

        Returns:
          list[str]: List of parameter values.

        Raises:
          TimeoutError: on a timeout.
        """
        values      = None
        values_cond = threading.Condition()

        def _get_parameters_cb(future):
            nonlocal values
            values = future.result().values
            with values_cond:
                values_cond.notify_all()

        self.get_parameters(names, _get_parameters_cb)
        with values_cond:
            if not values_cond.wait_for(lambda: values is not None,
                                        timeout_sec):
                raise TimeoutError()
        return [parameter_value_to_python(value) for value in values]

    def set_parameters_sync(self, param_tuples,
                            *, timeout_sec: Optional[float]=None):
        """ Set parameters given a list of parameters.
        The result after the returned future is complete
        will be of type ``rcl_interfaces.srv.SetParameters.Response``.

        Args:
          param_tuples: Sequence of parameters to set.
          timeout_sec: Timeout time in seconds waiting for setting remote
            parameters being completed. Wait forever, if `None`.

        Returns:
          Result of the request.

        Raises:
          TimeoutError: on a timeout.
        """
        results      = None
        results_cond = threading.Condition()

        def _set_parameters_cb(future):
            nonlocal results
            results = future.result().results
            with results_cond:
                results_cond.notify_all()

        parameters = [Parameter(param_tuple[0], value=param_tuple[1])
                      for param_tuple in param_tuples]
        self.set_parameters(parameters, _set_parameters_cb)
        with results_cond:
            if not results_cond.wait_for(lambda: results is not None,
                                         timeout_sec):
                raise TimeoutError()
        return results
