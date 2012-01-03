###############################################################################
# Copyright (c) 2011 Vanadium Labs LLC.                                       #
# All right reserved.                                                         #
#                                                                             #
# Redistribution and use in source and binary forms, with or without          #
# modification, are permitted provided that the following conditions are met: #
#                                                                             #
#   * Redistributions of source code must retain the above copyright notice,  #
#     this list of conditions and the following disclaimer.                   #
#   * Redistributions in binary form must reproduce the above copyright       #
#     notice, this list of conditions and the following disclaimer in the     #
#     documentation and/or other materials provided with the distribution.    #
#   * Neither the name of Vanadium Labs LLC nor the names of its contributors #
#     may be used to endorse or promote products derived from this software   #
#     without specific prior written permission.                              #
#                                                                             #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" #
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   #
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  #
# ARE DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT,   #
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES          #
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;#
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND #
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT  #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF    #
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.           #
###############################################################################

# Author: Michael Ferguson

import roslib; roslib.load_manifest('capabilities_server')
import rospy

import subprocess

from capabilities_utils import *

from capabilities_msgs.msg import *
from capabilities_msgs.srv import *

class CapabilitiesServer:

    def __init__(self):
        self.robot_type = rospy.get_param('robot/type')

        # list of capabilities
        self.capabilities = get_capabilities_list(self.robot_type)

        # node_name: [pkg/launch, pkg/launch] mapping
        self.node_launch = dict()

        # pkg/launch: [node_name, node_name] mapping
        self.launch_references = dict()

        # pkg/launch: subprocess mapping
        self.launch_files = dict()

        # ROS interface
        self.request_service = rospy.Service('request_capability', RequestCapability, self.request_cb)
        self.cancel_service = rospy.Service('cancel_capability', CancelCapability, self.cancel_cb)

    def request_cb(self, req):
        ''' Load required capabilities. '''
        response = RequestCapabilityResponse()
        response.launched = Capability()
        # find best match
        capability = self.find_match(req.request, req.use_approx)
        if capability == None:
            response.error_code.val = CapabilityErrorCodes.NO_MATCH_FOUND
            return response
        else:
            response.error_code.val = CapabilityErrorCodes.SUCCESS

        # Check if already running        
        pkg_launch = capability.launch_pkg + '/' + capability.launch_file
        if pkg_launch in self.launch_files.keys():
            self.launch_references.append(req.node_name)
            return response

        # Load the launch file
        proc = subprocess.Popen(['roscapable', capability.launch_pkg, capability.launch_file])
        self.launch_files[pkg_launch] = proc
        # update lists        
        self.launch_references[pkg_launch] = [req.node_name]
        if req.node_name in self.node_launch.keys():
            self.node_launch[req.node_name].append(pkg_launch)
        else:
            self.node_launch[req.node_name] = [pkg_launch]
        response.launched = capability
        return response

    def cancel_cb(self, req):
        ''' Cancel the need for a capability, reduce reference counts. '''
        response = CancelCapabilityResponse()
        response.error_code.val = CapabilityErrorCodes.NO_MATCH_FOUND
        try:
            launch_files = self.node_launch[req.node_name]
        except:
            return response
        for launch in launch_files:
            self.launch_references[launch].remove(req.node_name)
        del self.node_launch[req.node_name]
        response.error_code.val = CapabilityErrorCodes.SUCCESS
        return response

    def find_match(self, capability, use_approx):
        ''' Find match for requested capability. '''
        candidates = list()
        if capability.resource:
            rospy.loginfo('CAPABILITIES_SERVER: Matching based on resource name')
            for c in self.capabilities:
                if c.resource == capability.resource:
                    candidates.append(c)
        if capability.msg_pkg and capability.msg_type:
            rospy.loginfo('CAPABILITIES_SERVER: Matching based on message type')
            if len(candidates) > 0:
                for c in candidates:
                    if c.msg_pkg == capability.msg_pkg and c.msg_pkg == capability.msg_pkg:
                        pass
            else:
                for c in self.capabilities:
                    if c.msg_pkg == capability.msg_pkg and c.msg_pkg == capability.msg_pkg:
                        candidates.append(c)
        # TODO: match on keywords
        # TODO: determine best candidate
        print candidates
        return candidates[0]

    def cleanup(self):
        ''' Shutdown any unneeded launch files. '''
        shutdown = list()
        # TODO: Add timeouts for shutdown (to avoid driver thrashing)
        for launch in self.launch_references.keys():
            if len(self.launch_references[launch]) == 0:
                shutdown.append(launch)
        for launch in shutdown:
            # shutdown launch file
            self.launch_files[launch].terminate()
            del self.launch_files[launch]
            del self.launch_references[launch]
            rospy.loginfo('CAPABILITIES_SERVER: Shutdown %s', launch)

