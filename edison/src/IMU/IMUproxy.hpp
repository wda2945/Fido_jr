/*
 * IMUproxy.hpp
 * */

#ifndef IMUPROXY_HPP
#define IMUPROXY_HPP

#include <mutex>
#include <chrono>
#include "robot.h"
#include "IMUClass.hpp"

using namespace std::chrono;

class IMUproxy : public IMUClass
{
public:

	bool new_imu_data() override;

private:

	IMUproxy();
	~IMUproxy() {}

	void message_handler(const void *_msg, int len);

	std::mutex imuDataMutex;

	psImuPayload_t lastIMUmessage;
	std::chrono::system_clock::time_point IMUupdated;

	bool newIMUmessage = false;

	friend IMUproxy &the_imu_proxy();
	friend void IMU_message_handler(const void *_msg, int len);
};

IMUproxy &the_imu_proxy();

#endif



