string link_name        # child_link of the DetachableJoint
string child_model      # child_model of the DetachableJoint (what this joint holds on to)
string state            # attached | detached | unknown
bool   state_confirmed  # false when state is an assumption, not an observed event
bool   controllable     # false if the resolved attach/detach/state topics are not unique
