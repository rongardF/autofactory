string                    model_id        # unique world model name; must equal <model name=...> in the SDF
string                    sdf             # expanded SDF document
geometry_msgs/PoseStamped pose            # header.frame_id resolved via TF; empty => "world"
string                    attach_link     # "" => leave the SDF's joints as authored (assumed initially attached)
                                          # otherwise the joint whose child_link == attach_link stays attached and
                                          # all other joints of the model are detached after spawn
bool                      delete_with_child  # true => deleting a model that a joint of this model holds (child_model)
                                             # also deletes this model first, whatever its owner
