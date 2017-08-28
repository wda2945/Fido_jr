/*
 ============================================================================
 Name        : IMUproxy.c
 Author      : Martin
 Version     :
 Copyright   : (c) 2015 Martin Lane-Smith
 Description : Reads IMU
 ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "ps.h"
#include "robot.h"
#include "navigator/nav_debug.h"
#include "IMUproxy.hpp"

using namespace std;

void IMU_message_handler(const void *_msg, int len)
{
	the_imu_proxy().message_handler(_msg, len);
}

IMUproxy::IMUproxy()
{
	if (ps_subscribe(IMU_TOPIC, IMU_message_handler) != PS_OK)
	{
		ERRORPRINT("imu: Subscribe IMU_TOPIC failed");
	}
	else
	{
		DEBUGPRINT("imu: new");
	}
}

void IMUproxy::message_handler(const void *_msg, int len)
{
	const psMessage_t *msg = static_cast<const psMessage_t*>(_msg);

	if (msg->messageType == IMU_REPORT)
	{
		//critical section
	    unique_lock<mutex> lck {imuDataMutex};

		lastIMUmessage = msg->imuPayload;
		IMUupdated = std::chrono::system_clock::now();
		newIMUmessage = true;

//		DEBUGPRINT("imu: New IMU message");
	}
}

bool IMUproxy::new_imu_data()
{
	if (newIMUmessage && (IMUupdated + std::chrono::seconds(IMU_DATA_TIMEOUT)) > std::chrono::system_clock::now())
	{
		//critical section
	    unique_lock<mutex> lck {imuDataMutex};

		heading = lastIMUmessage.heading;
		pitch = lastIMUmessage.pitch;
		roll = lastIMUmessage.roll;
		updated = IMUupdated;

		newIMUmessage = false;
		return true;
	}
	return false;
}

IMUproxy &the_imu_proxy()
{
	static IMUproxy me;
	return me;
}

IMUClass &the_imu()
{
	return the_imu_proxy();
}
