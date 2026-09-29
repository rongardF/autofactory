# A single digital (boolean) I/O signal in normalized form.
string DIRECTION_IN  = "in"
string DIRECTION_OUT = "out"

string io_name        # unique logical name from config
string direction      # DIRECTION_IN | DIRECTION_OUT (matches IODirectionEnum)
bool   state          # current logical value (read) or desired value (command)
