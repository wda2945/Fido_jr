//
//  Options.h
//  FidoJr Edison
//
//  Created by Martin Lane-Smith on 6/14/14.
//  Copyright (c) 2014 Martin Lane-Smith. All rights reserved.
//

//optionmacro(name, var, min, max, def)

optionmacro("Motor Enable", motorEnable, 0, 1, 1)

optionmacro("Wait IMU", waitIMU, 0, 1, 1)
optionmacro("Wait Fix", waitGPS, 0, 1, 1)

optionmacro("Comms Stats", commStats, 0, 1, 0)

#include "autopilot/options.h"
#include "behavior/options.h"
#include "navigator/options.h"