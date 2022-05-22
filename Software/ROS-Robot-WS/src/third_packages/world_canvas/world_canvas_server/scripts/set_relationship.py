#!/usr/bin/env python

import rospy
import uuid
import unique_id
import world_canvas_msgs.srv

from rospy_message_converter import message_converter


if __name__ == '__main__':
    rospy.init_node('set_relationship')
    
    id      = rospy.get_param('~id')
    related = rospy.get_param('~relationship')
    action  = rospy.get_param('~action')

    rospy.loginfo("Waiting for set_relationship service...")
    rospy.wait_for_service('set_relationship')

    rospy.loginfo("%s relationship %s for annotation %s", "Add" if action == 1 else "Remove", related, id)
    set_rel_srv = rospy.ServiceProxy('set_relationship', world_canvas_msgs.srv.SetRelationship)
    response = set_rel_srv(unique_id.toMsg(uuid.UUID('urn:uuid:' + id)),
                           unique_id.toMsg(uuid.UUID('urn:uuid:' + related)), action)

    if response.result == True:
        rospy.loginfo("Relationship %s successfully %s. %s", related, "added" if action == 1 else "removed", response.message)
    else:
        rospy.logerr("%s relationship %s failed; %s", "Add" if action == 1 else "Remove", related, response.message)
