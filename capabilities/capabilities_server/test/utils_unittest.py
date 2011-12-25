#!/usr/bin/env python

import roslib; roslib.load_manifest('capabilities_server')
import rospy

import unittest

from capabilities_server.capabilities_utils import *

class TestUtils(unittest.TestCase):

    def test_easy(self):

        print get_capabilities_list('Maxwell')
        print get_capabilities_list('max')

        #self.assertAlmostEqual(numpy.linalg.norm(result-expected), 0.0, 6)
        #self.assertEqual(free_list[0], True)

if __name__ == '__main__':
    import rostest
    rostest.unitrun('capabilities_server', 'test_Utils', TestUtils, coverage_packages=['capabilities_server.utils'])

