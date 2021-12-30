# World canvas: a generic descriptor for a world, typically containing one or more geometric maps
# plus semantic information in the form of annotations.
#  - timestamp   : Creation/last update time
#  - name        : World name; it must be unique
#  - description : Informal description
#  - geometry    : 2D, 3D (just informative by now)

time timestamp
string name
string description
string geometry