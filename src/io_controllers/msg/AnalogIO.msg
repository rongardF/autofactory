# A single analog I/O signal in normalized form.
string DIRECTION_IN     = "in"
string DIRECTION_OUT    = "out"
string DOMAIN_VOLTAGE   = "voltage"
string DOMAIN_CURRENT   = "current"

string  io_name        # unique logical name from config
string  direction      # DIRECTION_IN | DIRECTION_OUT (matches IODirectionEnum)
float32 state          # current value (read) or desired value (command)
string  domain         # DOMAIN_VOLTAGE | DOMAIN_CURRENT (matches IODomainEnum)
