# Type of capability (See definitions below)
int32 type

# Message definition
string msg_pkg
string msg_type

# Resource names
string resource
string launch_pkg
string launch_file

# Keywords, ex. "base", "laser", etc
string[] keywords

# Capability types
int32 TYPE_DATA = 1
int32 TYPE_SERVICE = 2
int32 TYPE_ACTION = 3
