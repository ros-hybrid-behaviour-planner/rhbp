#!/usr/bin/env python2

from __future__ import division  # force floating point division when using plain /

import rospy
from rhbp_agent_taxi_all_cond_decoded_rhbp_Manager import \
    TaxiAgentManagerAllRHBPDecoded
from rhbp_agent_taxi_all_cond_rhbp_Manager import \
    TaxiAgentManagerAllRHBP

# This file start the different rhbp implementations regarding the configured parameter sim.
# It used to have more cases. Now only relevant are included
if __name__ == '__main__':
    try:
        rospy.init_node('agent_node', anonymous=True)
        sim = rospy.get_param('~sim', 7)  # default for debugging 'agentA1'
        if sim == 7:
            rhbp_agent = TaxiAgentManagerAllRHBP()
            rhbp_agent.set_manager()
        elif sim == 8:
            rhbp_agent = TaxiAgentManagerAllRHBPDecoded()
            rhbp_agent.set_manager()

        rhbp_agent.start_simulation()

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
