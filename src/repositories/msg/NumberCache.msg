# A cache entry holding a single numeric value (integers and decimals, signed).
string                    key                     # unique key within the station
builtin_interfaces/Time   valid_till              # absolute expiry instant
bool                      expire_at_valid_till    # if false, entry never expires
float64                   value                   # stored payload
