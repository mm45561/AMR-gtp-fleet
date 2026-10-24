# One GTP station as the fleet manager sees it (part of FleetState).
string station_id
string robot_id         # the AMR that holds the dock; '' = none
bool ready              # that AMR is docked and waiting: a dispatch would be accepted
string status           # for the screen: 'amr_1 is ready' or why it is not
