#!/usr/bin/env python

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

import roslib; roslib.load_manifest('capabilities_requester')
import rospy

import sys

from capabilities_server.capabilities_utils import *

from capabilities_msgs.msg import *
from capabilities_msgs.srv import *

class CapabilitiesRequester:

    def __init__(self, capabilities_file):
        self.capabilities = get_required_capabilities(capabilities_file)

        # request services
        rospy.wait_for_service('request_capability')
        request = rospy.ServiceProxy('request_capability', RequestCapability)
        self.cancel = rospy.ServiceProxy('cancel_capability', CancelCapability)
        for capability in self.capabilities:
            try:
                response = request( RequestCapabilityRequest(rospy.get_name(), capability, rospy.get_param('~use_approx', False)) )
                if response.error_code.val == CapabilityErrorCodes.SUCCESS:
                    rospy.loginfo('Successfully launched %s', response.launched)
                elif response.error_code.val == CapabilityErrorCodes.NO_MATCH_FOUND:
                    rospy.logerr('Unmet required capability')
                else:
                    rospy.logerr('Failed to launch capability')
            except rospy.ServiceException as e:
                print "Service did not process request: %s"%str(e)

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        response = self.cancel(rospy.get_name())
        print response.error_code.val

if __name__=='__main__':
    rospy.init_node('capabilities_requester')
    requester = CapabilitiesRequester(sys.argv[1])
    rospy.spin()

