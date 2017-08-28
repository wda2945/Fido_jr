//
//  software_profile.h
//  FidoJr Edison
//
//  Created by Martin Lane-Smith on 6/17/14.
//  Copyright (c) 2014 Martin Lane-Smith. All rights reserved.
//

#ifndef software_profile_h
#define software_profile_h

#define MOTOR_RESPONSE_WAIT_SECS	10
#define RADIUS_IN_MM				135

#ifndef M_PI
#define M_PI 3.1415927
#endif

#define SETTINGS_DOMAIN		"Edison Settings"
#define OPTIONS_DOMAIN		"Edison Options"
#define STATUS_DOMAIN		"Edison Status"
#define PROXIMITY_DOMAIN	"LIDAR"
#define ERRORS_DOMAIN		"Edison Errors"

#define FIDO_JR_LISTEN_PORT		7000
#define PING_PORT_NUMBER		5000

#define UART_PATH "/dev/ttyMFD1"

#define GPS_UART_PATH	"/dev/ttyACM0"
#define GPS_BAUDRATE	B9600

//paths
#define BEHAVIOR_TREE_CLASS	"/home/root/lua/behaviortree/behavior_tree.lua"
#define INIT_SCRIPT_PATH 	"/home/root/lua/init"				//initialization
#define BT_LEAF_PATH		"/home/root/lua/bt_leafs"
#define BT_ACTIVITY_PATH	"/home/root/lua/bt_activities"
#define HOOK_SCRIPT_PATH 	"/home/root/lua/hooks"				//Message hooks
#define GENERAL_SCRIPT_PATH "/home/root/lua/scripts"				//General scripts

#define WAYPOINT_FILE_PATH	"/home/root/data/waypoints.xml"
#define SAVED_SETTINGS_FILE "/home/root/data/settings.txt"
#define SAVED_OPTIONS_FILE  "/home/root/data/options.txt"

#endif //software_profile_h
