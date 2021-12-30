#!/usr/bin/env python

# This is a ros node wrapper for the mongod database server that
# sets various things using ROS parameters:
# - Parent namespace
#   - warehouse_host: hostname used by db
#   - warehouse_port: port used by db
# - Private namespace
#   - ~database location: where the db is stored.  Defaults to /tmp/db.
#   - ~overwrite: whether to overwrite existing db.  Defaults to false.

from __future__ import print_function
import roslib; roslib.load_manifest('warehouse_ros')
import rospy
import subprocess as sp
import sys
import os
from os import path
import shutil

def is_oneiric():
    if path.exists('/etc/issue'):
        rospy.logdebug('/etc/issue exists')
        with open('/etc/issue') as f:
            contents = f.readline()
            rospy.logdebug('contents are {0}'.format(contents))
            return '11.10' in contents
    else:
        rospy.logdebug('/etc/issue does not exist')



def is_lucid_or_maverick():
    if path.exists('/etc/issue'):
        rospy.logdebug('/etc/issue exists')
        with open('/etc/issue') as f:
            contents = f.readline()
            rospy.logdebug('contents are {0}'.format(contents))
            return '10.04' in contents or '10.10' in contents
    else:
        rospy.logdebug('/etc/issue does not exist')

def print_help_message():
    print("""
Usage: rosrun warehouse_ros_mongo mongo_wrapper_ros.py
Start the mongodb database server.  Configured using the following ROS parameters.  
    
Parameters in parent namespace
- warehouse_port: port used by db.  Defaults to 27017.
- warehouse_host: doesn't directly affect server, but used by clients to know where the db is.

Parameters in parent namespace
- db_path: where the db is stored on the filesystem.  Defaults to /tmp/db.
- database_path: same as db_path.  For backward compatibility.
- overwrite: whether to overwrite existing database if it exists.  Defaults to false.
""")

if __name__ == '__main__':

    if '--help' in sys.argv:
        print_help_message()
        sys.exit()

    rospy.init_node('mongodb')
    path_param = rospy.get_param('~database_path' , '/tmp/db')
    if path_param=='/tmp/db':
        path_param = rospy.get_param('~db_path' , '/tmp/db')
    dbpath = path.expanduser(path_param)
    overwrite = rospy.get_param('~overwrite', False)

    if '--repair' in sys.argv:
        rospy.loginfo("Repairing database")
        lock_file = '{0}/mongod.lock'.format(dbpath)
        if path.exists(lock_file):
            rospy.loginfo("  Removing lock file")
            os.remove(lock_file)
        sp.check_call(['mongodb', 'mongod', '--repair', '--dbpath', dbpath.format(dbpath)])
        rospy.loginfo("  Successfully repaired.")
        sys.exit(0)

    # The defaults here should match the ones used by each client library
    port = rospy.get_param('warehouse_port', 27017)
    host = rospy.get_param('warehouse_host', 'localhost')

    rospy.loginfo('Starting mongodb with db location {0} listening on {2}:{1}'.\
                  format(dbpath, port, host))

    if overwrite and path.exists(dbpath):
        shutil.rmtree(dbpath)
        rospy.loginfo('Removed existing db at %s', dbpath)

    if not path.exists(dbpath):
        rospy.loginfo('{0} did not exist; creating it.'.format(dbpath))
        os.makedirs(dbpath)




    # OK, now actually run mongod
    try:
        cmd = "mongod"
        sp.check_call("{2} --dbpath {0} --port {1}".\
                      format(dbpath, port, cmd).split())
    except sp.CalledProcessError as e:
        if e.returncode==12:
            rospy.loginfo("Ignoring mongod's nonstandard return code of 12")
        else:
            rospy.logerr("Mongo process exited with error code {0}".format(e.returncode))
    except OSError as e:
        rospy.logerr("Execution failed: %s", e)
  

