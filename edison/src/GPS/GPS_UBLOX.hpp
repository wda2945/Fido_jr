/*
 * GPS_UBLOX.hpp
 *
 *      Author: martin
 */

#ifndef GPS_UBLOX_H_
#define GPS_UBLOX_H_

#include <thread>
#include <mutex>
#include "robot.h"
#include "GPSClass.hpp"

class GPS_UBLOX : public GPSClass
{
public:
	GPS_UBLOX();
	~GPS_UBLOX(){};

	bool new_gps_data() override;

private:
	int FD {0};

	void GPS_thread_method();
	std::thread *gps_thread_handle {nullptr};
	std::mutex gpsDataMutex;

	struct {
		bool		dateValid;
		bool		timeValid;
		bool		positionValid;
		int32_t		lon;
		int32_t		lat;
		uint32_t	hAcc;		//horizontal accuracy
		bool		speedValid;
		int32_t		velN;
		int32_t		velE;
		int32_t		gSpeed;
		bool		headingValid;
		int32_t		headMot;
		uint32_t	sAcc;
		uint32_t	headAcc;
	} latest_fix;

	std::chrono::system_clock::time_point latest_update_time;
	std::chrono::system_clock::time_point latest_fix_time;

	bool newGPSdata {false};
	bool serial_init {false};
	bool gps_init {false};

	//////////////////////////////// UBX
	//UBX header sequence
	typedef enum {
		UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID, UBX_LENGTH_LSB, UBX_LENGTH_MSB, UBX_PAYLOAD
	} UBX_sequence_enum;

	#define UBX_SYNC1_BYTE	0xB5
	#define UBX_SYNC2_BYTE	0x62
	#define UBX_NAV_PVT		0x07
	#define UBX_NAV_STATUS	0x03

#define MAX_UBX_PAYLOAD	500
	typedef union {
		uint8_t payload[MAX_UBX_PAYLOAD];
		struct {
			uint32_t	iTOW;		//ms GPS Time of Week
			int16_t		year;
			uint8_t		month;
			uint8_t		day;
			uint8_t		hour;
			uint8_t		min;
			uint8_t		sec;
			uint8_t		valid;		//date & time validity flags
#define dateTimeValid	0x03
			uint32_t	tAcc;		//time accuracy
			int32_t		nano;		//fraction of second
			uint8_t		fixType;
			uint8_t		flags;
#define gnssFixOK 		0x01
#define validMag		0x08
#define headVehValid	0x20
#define carrSoln		0xC0
			uint8_t		flags2;
			uint8_t		numSV;		//satellites in view
			int32_t		lon;
			int32_t		lat;
#define UBX_LATLONG_SCALE	10000000
			int32_t		height;
			int32_t		hMSL;
			uint32_t	hAcc;		//horizontal accuracy
			uint32_t	vAcc;		//vertical accuracy
			int32_t		velN;
			int32_t		velE;
			int32_t		velD;
			int32_t		gSpeed;
			int32_t		headMot;
			uint32_t	sAcc;
			uint32_t	headAcc;
			uint16_t	pDOP;
			uint8_t		reserved1[6];
			int32_t		headVeh;
			int16_t		magDec;
			uint16_t	magAcc;
		}NAV_PVT;

		//other messages go here
	} UBX_t;

	UBX_t UBX_message;

	//NMEA
	// how long are max NMEA lines to parse?
	#define MAXLINELENGTH 200
	char currentline[MAXLINELENGTH];
	int lineidx {0};
	bool ParseNMEA();

	friend GPS_UBLOX &the_UBLOX_gps();
};

GPS_UBLOX &the_UBLOX_gps();

#define UBX_NAV 0x01 //Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
#define UBX_RXM 0x02 //Receiver Manager Messages: Satellite Status, RTC Status
#define UBX_INF 0x04 //Information Messages: Printf_Style Messages, with IDs such as Error, Warning, Notice
#define UBX_ACK 0x05 //Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages
#define UBX_CFG 0x06 //Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
#define UBX_UPD 0x09 //Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
#define UBX_MON 0x0A //Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
#define UBX_AID 0x0B //AssistNow Aiding Messages: Ephemeris, Almanac, other A_GPS data input
#define UBX_TIM 0x0D //Timing Messages: Time Pulse Output, Time Mark Results
#define UBX_ESF 0x10 //External Sensor Fusion Messages: External Sensor Measurements and Status Information
#define UBX_MGA 0x13 //Multiple GNSS Assistance Messages: Assistance data for various GNSS
#define UBX_LOG 0x21 //Logging Messages: Log creation, deletion, info and retrieval
#define UBX_SEC 0x27 //Security Feature Messages
#define UBX_HNR 0x28 //High Rate Navigation Results Messages: High rate time, position, speed, heading

extern const char *UBX_classnames[];

#endif
