# TrackedObject.msg
uint32 id             # Tracking ID from the filter
string class_name   # Classification: "UNKNOWN", "STATIC", "DYNAMIC"

# State of the object in the 'map' frame
geometry_msgs/Pose pose         # Position and orientation
geometry_msgs/Twist velocity    # Linear and angular velocity
geometry_msgs/Vector3 size      # Dimensions of the object (length, width, height)