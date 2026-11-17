string                      object_id
string                      mesh_path        # absolute STL path (meters); "" if primitives are used
string                      mesh_file_type   # default "stl"
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[]        primitive_poses
geometry_msgs/PoseStamped   pose             # frame_id = frame/link the object is placed in
string                      attach_link      # "" => world collision object
string[]                    touch_links      # only for attached objects
