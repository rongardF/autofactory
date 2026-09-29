# Stamped beam-interrupted state for a single Captron ORL2 laser cross output.
#
# header.stamp    : time the triggered state was evaluated and published
# header.frame_id : frame of the originating beam (propagated from the sensor)
# triggered       : True when the beam is interrupted (object at the TCP)
std_msgs/Header header
bool triggered
