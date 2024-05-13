std_msgs/Header header

# STATUS
uint8 OK = 200
uint8 NO_LOCALIZATION = 201
uint8 NO_TRAJECTORY = 201
uint8 ERROR = 203

uint8 status

# ERROR
uint8 UNKNOWN = 0
uint8 NO_PROJECTION = 1
uint8 UNREACHABLE_PROJECTION = 2
uint8 IMPOSSIBLE_BUILD_ARC = 3

uint8 error
