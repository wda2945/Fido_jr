//
//  Settings.h
//  Hex
//
//  Created by Martin Lane-Smith on 6/14/14.
//  Copyright (c) 2014 Martin Lane-Smith. All rights reserved.
//

//settingmacro(name, var, min, max, def)
        
//PID parameters

settingmacro("PID.Starting.Duty" ,pidStartingDuty, 0, 1, 0.1)
settingmacro("PID.Duty.Increment" ,pidDutyIncrement, 0, 1, 0.025)   //base of PID calculation
settingmacro("PID.Max.Increment"  ,pidMaxIncrement, 0, 1, 0.25)    //per PID_INTERVAL
settingmacro("PID.Speed.Factor"   ,pidSpeedFactor, 0, 10, 1.0)
settingmacro("PID.Median.Speed.Factor" ,pidMedianSpeedFactor, 0, 10, 0.0)
settingmacro("PID.Median.Distance.Factor" ,pidMedianDistanceFactor, 0, 10, 0.0)
settingmacro("PID.Heading.Error.Factor" ,pidHeadingErrorFactor, 0, 20, 5.0)
settingmacro("PID.Error.Total.Scale",pidErrorTotalScale, 1, 100, 20)

settingmacro("Turn.Ratio%", turnRatio, 0, 50, 25) 
        
settingmacro("Proximity.delay", proxDelay, 0, 10, 5)
        
//control limits
settingmacro("Arrival.Range", arrivalRange, 0, 200, 50)  //mm        for distance stop
settingmacro("Stop.Heading.Range", stopHeadingRange, 1, 30, 5)   //degrees   for heading stop
settingmacro("Hold.Heading.Range", holdHeadingRange, 1, 60, 25)  //degrees   for drift off heading stop