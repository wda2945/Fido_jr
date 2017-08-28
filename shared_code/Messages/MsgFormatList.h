/* 
 * File: MsgFormatList.h
 * 			Fido_jr
 * Author: martin
 *
 * Created on March 23, 2014, 1:11 PM
 */


//formatmacro(enum, type, var, size)
//-ve length for variable length names

formatmacro(PS_TICK_PAYLOAD, psTickPayload_t, tickPayload, sizeof(psTickPayload_t))

//sensors
formatmacro(PS_IMU_PAYLOAD,psImuPayload_t,imuPayload,sizeof(psImuPayload_t))
formatmacro(PS_ODOMETRY_PAYLOAD,psOdometryPayload_t,odometryPayload,sizeof(psOdometryPayload_t))
        
//motors
formatmacro(PS_MOTOR_COMMAND_PAYLOAD, psMotorCommandPayload_t, motorCommand, sizeof(psMotorCommandPayload_t))
formatmacro(PS_MOTOR_RESPONSE_PAYLOAD, psMotorResponsePayload_t, motorResponse, sizeof(psMotorResponsePayload_t))

//sound
formatmacro(PS_SPEECH_PAYLOAD,psSpeechPayload_t,speechPayload,sizeof(psSpeechPayload_t))
formatmacro(PS_BUZZ_PAYLOAD,psBuzzPayload_t,buzzPayload,sizeof(psBuzzPayload_t))
        