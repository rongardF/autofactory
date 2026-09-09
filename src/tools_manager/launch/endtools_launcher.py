#TODO: implement a Python ROS2 launch file that
# - accepts the 'tool_rack' config file and parses it into YAML
# - loops over each endtool in 'slots' and 
#   - launches a separate endtool model spawner
#   - launches each endtool node using the corresponding endtools launch file (does not touch their lifecycle state)
# - it will use the 'get_tool_info' function (from endtools package) to get the endtool launch file and model path for each endtool