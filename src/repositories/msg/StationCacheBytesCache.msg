# A cache entry holding a raw byte buffer.
string                    key                     # unique key within the station
builtin_interfaces/Time   valid_till              # absolute expiry instant
bool                      expire_at_valid_till    # if false, entry never expires
uint8[]                   value                   # stored payload
