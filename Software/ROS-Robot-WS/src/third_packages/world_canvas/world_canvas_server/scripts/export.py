#!/usr/bin/env python

import rospy
import world_canvas_msgs.srv


if __name__ == '__main__':
    rospy.init_node('export')
    
    file = rospy.get_param('~file')

    rospy.loginfo("Waiting for yaml_export service...")
    rospy.wait_for_service('yaml_export')

    rospy.loginfo("Export annotations to file '%s'", file)
    export_srv = rospy.ServiceProxy('yaml_export', world_canvas_msgs.srv.YAMLExport)
    response = export_srv(file)

    if response.result == True:
        rospy.loginfo("Database successfully exported to file '%s'", file)
    else:
        rospy.logerr("Export database failed; %s", response.message)
