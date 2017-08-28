//
//  MsgMacros.h
//  Common
//
//  Created by Martin Lane-Smith on 1/25/14.
//  Copyright (c) 2013 Martin Lane-Smith. All rights reserved.
//

//#define messagemacro(enum, qos, topic, payload, long name)

messagemacro(TICK,2,TICK_TOPIC,PS_TICK_PAYLOAD,"Tick")
        
//raw sensors (sensor -> navigator)
messagemacro(IMU_REPORT,3,IMU_TOPIC,PS_IMU_PAYLOAD,"IMU")
messagemacro(ODOMETRY,2,ODO_TOPIC,PS_ODOMETRY_PAYLOAD,"Odometry")

//raw movement -> motors
messagemacro(MOTOR_ABORT,1,MOT_ACTION_TOPIC,PS_NO_PAYLOAD,"Motor Abort")
messagemacro(MOTOR_ACTION,1,MOT_ACTION_TOPIC,PS_MOTOR_COMMAND_PAYLOAD,"Motor Action")
//motor response
messagemacro(MOTOR_RESPONSE,1,MOT_RESPONSE_TOPIC,PS_MOTOR_RESPONSE_PAYLOAD,"Motor Response")
        
//sound
messagemacro(SPEAK,2,SOUND_TOPIC,PS_SPEECH_PAYLOAD,"Speak")
messagemacro(BUZZ,2,SOUND_TOPIC,PS_BUZZ_PAYLOAD,"Buzz")
        
//#undef messagemacro
