#!/usr/bin/env python

import rospy
import world_canvas_msgs.srv


if __name__ == '__main__':
    rospy.init_node('import')
    
    file = rospy.get_param('~file')

    rospy.loginfo("Waiting for yaml_import service...")
    rospy.wait_for_service('yaml_import')

    rospy.loginfo("Import annotations from file '%s'", file)
    import_srv = rospy.ServiceProxy('yaml_import', world_canvas_msgs.srv.YAMLImport)
    response = import_srv(file)

    if response.result == True:
        rospy.loginfo("Database successfully imported from file '%s'", file)
    else:
        rospy.logerr("Import database failed; %s", response.message)

