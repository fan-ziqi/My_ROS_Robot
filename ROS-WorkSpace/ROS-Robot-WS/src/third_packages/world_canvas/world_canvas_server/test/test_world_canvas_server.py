#!/usr/bin/env python

import sys
import rospy
import unittest
from nav_msgs.msg import *
from nav_msgs.srv import *
from map_store.srv import *
 
## Tests map store my reading a map from the map topic,
## reading a map from the dynamic_map service, renaming
## both maps, republishing both maps, and deleting both
## maps.
class TestMapStore(unittest.TestCase):
    ## test 1 == 1
    def compare_maps(self, map_a, map_b):
        return str(map_a) == str(map_b)

    def create_map(self, d):
        test_map = OccupancyGrid()
        test_map.info.resolution = 1.0 + d
        test_map.info.width = 10
        test_map.info.height = 10
        test_map.info.origin.position.x = 0.0 + d
        test_map.info.origin.position.y = 1.0 + d
        test_map.info.origin.position.z = 2.0 + d
        test_map.info.origin.orientation.x = 3.0 + d
        test_map.info.origin.orientation.y = 4.0 + d
        test_map.info.origin.orientation.z = 5.0 + d
        test_map.info.origin.orientation.w = 6.0 + d
        test_map.data = []
        for i in range(0, 100):
            test_map.data.append(i)
        return test_map

    def on_map(self, msg):
        if (self.wait_for_map and self.check_map != None):
            #self.assertEquals(self.compare_maps(self.check_map, msg), True, "Invalid map return")
            self.wait_for_map = False
        
        

    def test_annotations_store(self):
        rospy.init_node('test_annotations_store')

        #Create the two test maps
        test_map_1 = self.create_map(00.0)
        test_map_2 = self.create_map(10.0)
        test_map_2.data.reverse()

        self.wait_for_map = False
        self.check_map = None
        
        #Create services, publishers, and subscribers
        map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=5)
        test_map_sub = rospy.Subscriber('/test_map', OccupancyGrid, self.on_map)
        dynamic_map = rospy.Service('dynamic_map', GetMap, lambda x: GetMapResponse(map=test_map_2))
        print "Wait for /list_maps"
        rospy.wait_for_service("/list_maps")
        list_maps = rospy.ServiceProxy('/list_maps', ListMaps)
        print "Wait for /name_latest_map"
        rospy.wait_for_service("/save_map")
        name_latest_map = rospy.ServiceProxy('/save_map', SaveMap)
        print "Wait for /delete_map"
        rospy.wait_for_service("/delete_map")
        delete_map = rospy.ServiceProxy('/delete_map', DeleteMap)
        print "Wait for /rename_map"
        rospy.wait_for_service("/rename_map")
        rename_map = rospy.ServiceProxy('/rename_map', RenameMap)
        print "Wait for /publish_map"
        rospy.wait_for_service("/publish_map")
        publish_map = rospy.ServiceProxy('/publish_map', PublishMap)
        print "Wait for /test_dynamic_map"
        rospy.wait_for_service("/test_dynamic_map")
        test_dynamic_map = rospy.ServiceProxy('/test_dynamic_map', GetMap)


        print "Wait 1 second for everything to start up"
        rospy.sleep(1.0)

        #Get the initial list of all maps
        initial_map_list = []
        for m in list_maps().map_list:
            initial_map_list.append(m.map_id)
        print "Initial maps:", initial_map_list

        #Write out map 1
        print "Sending map 1"
        map_pub.publish(test_map_1)
        rospy.sleep(5.0)

        #Get the first map
        map_id_1 = None
        for i in list_maps().map_list:
            if not i.map_id in initial_map_list:
                self.assertEquals(map_id_1, None, "Two or more maps from /map topic")
                map_id_1 = i.map_id
        print "First map is:", map_id_1
        self.assertNotEquals(map_id_1, None, "Map was not loaded from the /map topic")
        
        #Save the second map
        saved_map_name = "test_map_srv"
        name_latest_map(map_name=saved_map_name)
        rospy.sleep(5.0)

        #Get the second map
        map_id_2 = None
        for i in list_maps().map_list:
            if not i.map_id in initial_map_list and i.map_id != map_id_1:
                self.assertEquals(map_id_2, None, "Two or more maps from dynamic_map")
                self.assertEquals(i.name, saved_map_name, \
                                      "Saved map has the wrong name: %s instead of %s"%(i.name, saved_map_name))
                map_id_2 = i.map_id
        print "Second map is:", map_id_2
        self.assertNotEquals(map_id_2, None, "Map was not loaded from the dynamic_map")
        
        #Re-name the first map
        print "Renaming first map"
        topic_map_name = "test_map_msg"
        rename_map(map_id=map_id_1, new_name=topic_map_name)
        rospy.sleep(1.0)

        #Ensure the renaming happened
        for i in list_maps().map_list:
            if i.map_id == map_id_1:
                self.assertEquals(i.name, topic_map_name, \
                                      "Saved map has the wrong name: %s instead of %s"%(i.name, topic_map_name))
        
        
        #Display both maps
        print "Displaying first map"
        self.wait_for_map = True
        self.check_map = test_map_1
        publish_map(map_id_1)
        while (self.wait_for_map):
            rospy.sleep(1.0)
        self.assertEquals(self.compare_maps(test_dynamic_map().map, test_map_1), True, "Test map 1 is bad")
        
        print "Displaying second map"
        self.wait_for_map = True
        self.check_map = test_map_2
        publish_map(map_id_2)
        while (self.wait_for_map):
            rospy.sleep(1.0)
        self.assertEquals(self.compare_maps(test_dynamic_map().map, test_map_2), True, "Test map 2 is bad")
        
        #Delete both maps
        print "Deleting both maps"
        delete_map(map_id_1)
        delete_map(map_id_2)
        rospy.sleep(1.0)
        
        #Check that they are gone
        print "Ensuring that the maps are gone"
        for i in list_maps().map_list:
            self.assertNotEquals(i.map_id, map_id_1, "The /map topic map could not be deleted")
            self.assertNotEquals(i.map_id, map_id_2, "The dynamic_map map could not be deleted")

        
        #rospy.spin()
        rospy.sleep(1.0)
        print "Finished"




if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_annotations_store', TestMapStore)
