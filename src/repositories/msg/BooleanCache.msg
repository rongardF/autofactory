# A cache entry holding a single boolean value.
string                    key                     # unique key within the station
builtin_interfaces/Time   valid_till              # absolute expiry instant
bool                      expire_at_valid_till    # if false, entry never expires
bool                      value                   # stored payload
