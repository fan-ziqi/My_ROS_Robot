#!/usr/bin/env python

import rospy
import uuid
import unique_id
import world_canvas_msgs.srv

from rospy_message_converter import message_converter


if __name__ == '__main__':
    rospy.init_node('set_keyword')
    
    id      = rospy.get_param('~id')
    keyword = rospy.get_param('~keyword')
    action  = rospy.get_param('~action')

    rospy.loginfo("Waiting for set_keyword service...")
    rospy.wait_for_service('set_keyword')

    rospy.loginfo("%s keyword %s for annotation %s", "Add" if action == 1 else "Remove", keyword, id)
    set_kw_srv = rospy.ServiceProxy('set_keyword', world_canvas_msgs.srv.SetKeyword)
    response = set_kw_srv(unique_id.toMsg(uuid.UUID('urn:uuid:' + id)), keyword, action)

    if response.result == True:
        rospy.loginfo("Keyword %s successfully %s. %s", keyword, "added" if action == 1 else "removed", response.message)
    else:
        rospy.logerr("%s keyword %s failed; %s", "Add" if action == 1 else "Remove", keyword, response.message)
