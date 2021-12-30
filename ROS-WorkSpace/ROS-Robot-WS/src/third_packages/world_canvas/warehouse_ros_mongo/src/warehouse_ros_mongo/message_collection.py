# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Bhaskara Marthi

# Collection of messages stored in a Mongo table and GridFS

import pymongo as pm
import gridfs as gfs
import rospy
import StringIO
import std_msgs.msg
import json
import bson.json_util

class MessageCollection:

    def __init__(self, db, coll, msg_class,
                 db_host=None, db_port=None, indexes=[]):
        """
        @param db: Name of database
        @param coll: Name of collection
        @param indexes: List of fields to build indexes on.
        @param msg_class: The class of the message object being stored
        @param db_host: The host where the db server is listening.
        @param db_port: The port on which the db server is listening.

        Creates collection, db, and indexes if don't already exist.
        The database host and port are set to the provided values if given.
        If not, the ROS parameters warehouse_host and warehouse_port are used,
        and these in turn default to localhost and 27017.
        """

        # Connect to mongo
        self.host = db_host or rospy.get_param('warehouse_host', 'localhost')
        self.port = db_port or rospy.get_param('warehouse_port', 27017)
        while not rospy.is_shutdown():
            try:
                self.conn = pm.Connection(self.host, self.port)
                break
            except:
                rospy.loginfo( "Attempting to connect to mongodb @ {0}:{1}".\
                               format(self.host,self.port))
                rospy.sleep(2.0)

        # Set up db, collection, gridfs
        self.db = self.conn[db]
        self.coll = self.db[coll]
        self.fs = gfs.GridFS(self.db)
        self.msg_class = msg_class

        # Indexes
        for ind in indexes:
            self.ensure_index(ind)
        self.ensure_index('creation_time')

        # Add to the metatable

        # Set up insertion pub
        insertion_topic = 'warehouse/{0}/{1}/inserts'.format(db, coll)
        self.insertion_pub = rospy.Publisher(insertion_topic, std_msgs.msg.String,
                                             latch=True, queue_size=5)
        

    def ensure_index(self, ind, **kwargs):
        info = self.coll.index_information()
        if ind in info:
            rospy.logdebug("Index {0} already exists".format(ind))
        else:
            kwargs['name'] = ind
            self.coll.ensure_index(ind, **kwargs)


    def insert(self, m, metadata={}, **kwargs):
        """
        @param m: Message to insert
        @param metadata: Dictionary of metadata to associate with message
        """
        # Insert raw message into gridFS
        buff = StringIO.StringIO()
        m.serialize(buff)
        v = buff.getvalue()
        msg_id = self.fs.put(v)

        # Create db entry
        entry= metadata.copy()
        entry['blob_id'] = msg_id
        entry['creation_time'] = rospy.Time.now().to_sec()

        # Insert message info
        self.coll.insert(entry, **kwargs)

        # Publish ros notification
        s = json.dumps(entry, default=bson.json_util.default)
        self.insertion_pub.publish(s)


    def query(self, query, metadata_only=False, sort_by='', ascending=True):
        """
        Perform a query.

        @return: Iterator over tuples (message, metadata) if metadata_only is
        False, or iterator over metadata if it's true
        """
        if sort_by:
            results = self.coll.find(query, sort=[(sort_by, pm.ASCENDING if
                ascending else pm.DESCENDING)])
        else:
            results = self.coll.find(query)
            
        if metadata_only:
            return results
        else:
            return (self.process_entry(r) for r in results)

    def find_one(self, query, metadata_only=False, sort_by='', ascending=True):
        """
        Like query except returns a single matching item, or None if
        no item exists
        """
        return next(self.query(query, metadata_only, sort_by, ascending), None)

    def remove(self, query):
        "Remove items matching query and return number of removed items."
        num_removed = 0
        for item in self.query(query, metadata_only=True):
            self.coll.remove(item['_id'])
            num_removed += 1
            self.fs.delete(item['blob_id'])
        return num_removed
        
    def process_entry(self, r):
        blob = self.fs.get(r['blob_id'])
        msg = self.msg_class()
        msg.deserialize(blob.read())
        return msg, r

    def update(self, entry, metadata=None, msg=None):
        """
        Update a message and/or metadata.

        @param entry: The existing metadata entry
        @param metadata: Updates to metadata.  These are merged with the existing dictionary entries.
        @param msg: If specified, a new message object to store in place of the current one.
        """
        old_blob_id = None
        if msg:
            buf = StringIO.StringIO()
            msg.serialize(buf)
            v = buf.getvalue()
            new_msg_id = self.fs.put(v)
            old_blob_id = entry['blob_id']
            entry['blob_id'] = new_msg_id
        if metadata:
            entry.update(metadata)
        self.coll.save(entry, safe=True)
        if old_blob_id:
            self.fs.delete(old_blob_id)

    def count(self):
        return self.coll.count()

            
        
        

    
        
              
