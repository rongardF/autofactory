# A cache entry holding an array of string values.
string                    key                     # unique key within the station
builtin_interfaces/Time   valid_till              # absolute expiry instant
bool                      expire_at_valid_till    # if false, entry never expires
string[]                  value                   # stored payload
