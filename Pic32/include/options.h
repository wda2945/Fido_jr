//
//  Options.h
//  Hex
//
//  Created by Martin Lane-Smith on 6/14/14.
//  Copyright (c) 2014 Martin Lane-Smith. All rights reserved.
//

//optionmacro(name, var, min, max, def)

optionmacro("Motor.Enable", motorEnable, 0, 1, 1)
optionmacro("Edison.Power", edisonPower, 0, 1, 1)
optionmacro("Print.Stats", printStats, 0, 1, 0)

optionmacro("Calibrate.IMU", calibrateIMU, 0, 1, 0)        
        
optionmacro("Front.Left.Prox", FLenable, 0, 1, 1)
optionmacro("Front.Center.Prox", FCenable, 0, 1, 1)
optionmacro("Front.Right.Prox", FRenable, 0, 1, 1)
optionmacro("Rear.Left.Prox", RLenable, 0, 1, 1)
optionmacro("Rear.Center.Prox", RCenable, 0, 1, 1)
optionmacro("Rear.Right.Prox", RRenable, 0, 1, 1)