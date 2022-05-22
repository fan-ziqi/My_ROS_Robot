# Annotation: a generic descriptor for an element (object) in a semantic map
# An annotation can be used to introspect, visualize or key into database filters/searches without
# having to touch the described object directly
#  - timestamp : Creation/last update time
#  - world     : World the object belongs to
#  - id        : Annotation unique id
#  - data_id   : Referenced object unique id (an object can be reference by 1 or more annotations)
#  - name      : Referenced object name
#  - type      : Referenced object type (a message type, as world canvas objects are ROS messages)
#  - shape     : One of 1 (CUBE), 2 (SPHERE), 3 (CYLINDER), 9 (TEXT)
#  - color     : R, G, B, A (optional)
#  - size      : X, Y, Z (optional)
#  - keywords  : Generic properties of this object: allows basic filtering without introspecting
#                the object itself
#  - relationships : List of associated annotations, used for retrieving operational sets of objects

# General properties
time timestamp
uuid_msgs/UniqueID id
uuid_msgs/UniqueID data_id
string world
string name
string type

# Physical properties
int32  shape
std_msgs/ColorRGBA color
geometry_msgs/Vector3 size
geometry_msgs/PoseWithCovarianceStamped pose

# Querying properties
string[] keywords
uuid_msgs/UniqueID[] relationships
