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

import rospkg

import os
import yaml
from capabilities_msgs.msg import Capability

def parse_capability(desc):
    ''' Parse a Capability message from yaml description. '''
    c = Capability()
    if 'type' in desc.keys():
        c.type = desc['type']
    if 'msg_pkg' in desc.keys():
        c.msg_pkg = desc['msg_pkg']
    if 'msg_type' in desc.keys():
        c.msg_type = desc['msg_type']
    if 'resource' in desc.keys():
        c.resource = desc['resource']
    if 'keywords' in desc.keys():
        c.keywords = desc['keywords']
    return c


def list_capabilities(robot, pkg, path, capabilities):
    ''' Scan directory, recursively adding capabilities to dictionary. '''
    for file in os.listdir(path):
        f = os.path.join(path, file)
        if os.path.isfile(f):
            file_info = f.split('.')
            if file_info[-1] == 'capability':
                try:
                    desc = yaml.load(open(f))
                except Exception as e:
                    print e
                    continue
                if 'robots' in desc.keys():
                    if not robot in desc['robots']:
                        continue
                for x in desc['capabilities_offered']:
                    c = parse_capability(x)
                    c.launch_pkg = pkg
                    c.launch_file = file.replace('.capability','.launch')
                    capabilities.append(c)
        elif os.path.isdir(f):
            list_capabilities(robot, pkg, f, capabilities)
    return capabilities


def get_required_capabilities(capabilities_file):
    ''' Get a list of required capabilities. '''
    try:
        desc = yaml.load(open(capabilities_file))
    except Exception as e:
        print e
        return []
    capabilities = list()
    for x in desc['capabilities_required']:
        capabilities.append(parse_capability(x))
    return capabilities


def get_capabilities_list(robot_type):
    ''' Returns a dictionary representing known
        capabilities installed on the system. '''

    pack = rospkg.RosPack()
    packages = pack.list()

    capabilities_list = list()

    for pkg in packages:
        path = pack.get_path(pkg)
        list_capabilities(robot_type, pkg, path, capabilities_list)

    return capabilities_list


