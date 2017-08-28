/*
 ============================================================================
 Name        : GPS_UBLOX.cpp
 Author      : Martin
 Version     :
 Copyright   : (c) 2015 Martin Lane-Smith
 Description : Reads remote GPS
 ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>

#include "software_profile.h"
#include "GPS_UBLOX.hpp"

#include <mutex>

#include "ps.h"


//#define DEBUGPRINT(...) tprintf( __VA_ARGS__);tfprintf(ubxLogFile, __VA_ARGS__);

#define DEBUGPRINT(...) tfprintf(ubxLogFile, __VA_ARGS__);
#define ERRORPRINT(...) tprintf(__VA_ARGS__);tfprintf(ubxLogFile, __VA_ARGS__);

FILE *ubxLogFile;

GPS_UBLOX::GPS_UBLOX()
{
	ubxLogFile = fopen_logfile("ublox");

	//Create thread
	gps_thread_handle = new std::thread([this](){GPS_thread_method();});
}

void GPS_UBLOX::GPS_thread_method()
{

	try
	{
		//open serial port
		struct termios settings;

		//initialize UART
		while ((FD = open(GPS_UART_PATH, O_RDWR | O_NOCTTY)) < 0)
		{
			ERRORPRINT("gps: failed to open %s - %s",  GPS_UART_PATH, strerror(errno));
			ps_set_condition(GPSTTY_NF_ERROR);
			sleep(5);
		}
		ps_cancel_condition(GPSTTY_NF_ERROR);

		while (!serial_init)
		{
			if (tcgetattr(FD, &settings) < 0)
			{
				ERRORPRINT("gps: tcgetattr fail %s - %s", GPS_UART_PATH, strerror(errno));
				ps_set_condition(GPSTTY_INIT_ERROR);
			}
			else
			{
				//no processing
				settings.c_iflag = 0;
				settings.c_oflag = 0;
				settings.c_lflag = 0;
				settings.c_cflag = CLOCAL | CREAD | CS8;        //no modem, 8-bits

				if (GPS_BAUDRATE)
				{
					//baudrate
					if ((cfsetospeed(&settings, GPS_BAUDRATE) < 0) || (cfsetispeed(&settings, GPS_BAUDRATE) < 0))
					{
						ERRORPRINT("gps: cfsetospeed fail %s - %s", GPS_UART_PATH, strerror(errno));
						ps_set_condition(GPSTTY_INIT_ERROR);
					}
					else
					{
						if (tcsetattr(FD, TCSANOW, &settings) < 0)
						{
							ERRORPRINT("gps: tcgetattr fail %s - %s", GPS_UART_PATH, strerror(errno));
							ps_set_condition(GPSTTY_INIT_ERROR);
						}
						else
						{
							serial_init = true;
						}
					}
				}
			}
			if (!serial_init) sleep(1);
		}
		ps_cancel_condition(GPSTTY_INIT_ERROR);
		DEBUGPRINT("gps: serial (%s) port ready.", GPS_UART_PATH);

		//initialize & configure GPS

		gps_init = true;
		ps_set_condition(GPS_ONLINE);

		while (1)
		{
			//read a message
			uint8_t c;
			int chars_read;
			do
			{
				chars_read = read(FD, &c, 1);
			} while (chars_read == 0);

			switch(c)
			{
			case '$':
			{
				//start of NMEA message - ignored
				lineidx = 0;
				currentline[lineidx++] = c;

				while (lineidx != 0)
				{
					//next character
					do
					{
						chars_read = read(FD, &c, 1);
					} while (chars_read == 0);

					if (c == '\n') {
						currentline[lineidx] = 0;
						lineidx = 0;

					} else {

						currentline[lineidx++] = c;
						if (lineidx >= MAXLINELENGTH) {
							lineidx = MAXLINELENGTH - 1;
							currentline[lineidx] = 0;
							lineidx = 0;
						}
					}
				}

				if (strstr(currentline, "$GNTXT"))
				{
					DEBUGPRINT("nmea: %s", currentline);
				}
				else
				{
					currentline[6] = '\0';
					DEBUGPRINT("nmea: %s ignored", currentline);
				}
			}
			break;
			case UBX_SYNC1_BYTE:
			{
				//start of UBX Message
				bool UBX_complete {false};
				bool UBX_good {false};
				UBX_sequence_enum UBX_header_seq = UBX_SYNC2;
				uint32_t payload_length {0};
				uint32_t payload_index {0};
				uint32_t payload_count {0};
				uint8_t ubx_class {0};
				uint8_t ubx_id {0};

				while (!UBX_complete)
				{
					do
					{
						chars_read = read(FD, &c, 1);
					} while (chars_read == 0);

					switch(UBX_header_seq)
					{
					case UBX_SYNC2:
						if (c != UBX_SYNC2_BYTE) UBX_complete = true;
						UBX_header_seq = UBX_CLASS;
						break;
					case UBX_CLASS:
						ubx_class = c;
						UBX_header_seq = UBX_ID;
						break;
					case UBX_ID:
						ubx_id = c;
						UBX_header_seq = UBX_LENGTH_LSB;
						break;
					case UBX_LENGTH_LSB:
						payload_length = c;
						UBX_header_seq = UBX_LENGTH_MSB;
						break;
					case UBX_LENGTH_MSB:
						payload_length |= (c << 8);
						if (payload_length > MAX_UBX_PAYLOAD)
						{
							ERRORPRINT("ubx: UBX-%s-0x%x payload too long: %d", UBX_classnames[ubx_class], ubx_id, payload_length);
							UBX_complete = true;
						}
						else
						{
							payload_count = payload_length + 2;
						}
						UBX_header_seq = UBX_PAYLOAD;
						break;
					case UBX_PAYLOAD:
					default:
					{
						UBX_message.payload[payload_index++] = c;
						if (--payload_count == 0)
						{
							UBX_complete = true;
							UBX_good = true;
						}
					}
					break;
					}
				}

				if (UBX_good)
				{
					//checksum
					uint8_t CK_A = 0;
					uint8_t CK_B = 0;

					CK_A = CK_A + ubx_class;
					CK_B = CK_B + CK_A;
					CK_A = CK_A + ubx_id;
					CK_B = CK_B + CK_A;
					CK_A = CK_A + (payload_length & 0xff);
					CK_B = CK_B + CK_A;
					CK_A = CK_A + ((payload_length >> 8) & 0xff);
					CK_B = CK_B + CK_A;

					for (uint32_t i=0; i<payload_length; i++)
					{
						CK_A = CK_A + UBX_message.payload[i];
						CK_B = CK_B + CK_A;
					}
					uint8_t PL_CK_A = UBX_message.payload[payload_length];
					uint8_t PL_CK_B = UBX_message.payload[payload_length+1];

					if ((CK_A == PL_CK_A) && (CK_B == PL_CK_B))
					{
						//process message
						switch (ubx_class)
						{
						case UBX_NAV:
							switch (ubx_id)
							{
							case UBX_NAV_PVT:
							{
								DEBUGPRINT("UBX: NAV-PVT");

								year 				= UBX_message.NAV_PVT.year;
								month 				= UBX_message.NAV_PVT.month;
								day 				= UBX_message.NAV_PVT.day;
								hour 				= UBX_message.NAV_PVT.hour;
								min 				= UBX_message.NAV_PVT.min;
								sec 				= UBX_message.NAV_PVT.sec;
								numSatsInView 		= UBX_message.NAV_PVT.numSV;

								if ((UBX_message.NAV_PVT.valid & dateTimeValid) == dateTimeValid)
								{
									dateValid = true;
									timeValid = true;
								}
								else
								{
									dateValid = false;
									timeValid = false;
								}

								if (validMag & UBX_message.NAV_PVT.flags)
								{
									magvariation	= UBX_message.NAV_PVT.magDec;
									magValid		= true;
								}
								else
								{
									magValid		= false;
								}

								latest_fix.lon 		= UBX_message.NAV_PVT.lon;
								latest_fix.lat 		= UBX_message.NAV_PVT.lat;
								latest_fix.hAcc 	= UBX_message.NAV_PVT.hAcc;
								latest_fix.velN 	= UBX_message.NAV_PVT.velN;
								latest_fix.velE 	= UBX_message.NAV_PVT.velE;
								latest_fix.gSpeed 	= UBX_message.NAV_PVT.gSpeed;
								latest_fix.headMot 	= UBX_message.NAV_PVT.headMot / 100000;
								latest_fix.sAcc 	= UBX_message.NAV_PVT.sAcc;
								latest_fix.headAcc 	= UBX_message.NAV_PVT.headAcc / 100000;

								DEBUGPRINT("%02d:%02d:%02d %f, %f (+/- %f m)", \
										UBX_message.NAV_PVT.hour, UBX_message.NAV_PVT.min, UBX_message.NAV_PVT.sec,\
										(float) UBX_message.NAV_PVT.lon/UBX_LATLONG_SCALE, \
										(float) UBX_message.NAV_PVT.lat/UBX_LATLONG_SCALE, \
										(float) UBX_message.NAV_PVT.hAcc/1000);


								bool current_fix_status = fix;
								latest_update_time = std::chrono::system_clock::now();

								if (gnssFixOK & UBX_message.NAV_PVT.flags)
								{
									fix = true;
									if (!current_fix_status)
									{
										ps_set_condition(GPS_FIX);
										DEBUGPRINT("ubx: gained fix");
									}
									newGPSdata = true;
									latest_fix_time = std::chrono::system_clock::now();
									latest_fix.positionValid = true;
									latest_fix.speedValid = true;

									if (headVehValid & UBX_message.NAV_PVT.flags)
									{
										latest_fix.headingValid = true;
									}
									else
									{
										latest_fix.headingValid = false;
									}
									if ((carrSoln & UBX_message.NAV_PVT.flags) > 0)
									{
										dgps_data = true;
									}
									else
									{
										dgps_data = false;
									}
									DEBUGPRINT("ubx: %s%s%s%s%s",
											(fix ? "fix," : "no fix"),
											(latest_fix.positionValid ? ", position valid," : ""),
											(dgps_data ? ", dgps data" : ""),
											(latest_fix.headingValid ? ", heading valid" : ""),
											(latest_fix.speedValid ? ", speed valid" : ""))
								}
								else
								{
									fix = false;
									if (current_fix_status)
									{
										ps_cancel_condition(GPS_FIX);
										DEBUGPRINT("ubx: lost fix");
									}
								}

							}
							break;
							case UBX_NAV_STATUS:
								DEBUGPRINT("UBX: NAV-STATUS");
								break;
							//other NAV messages go here
							default:
								DEBUGPRINT("ubx: UBX-NAV-0x%x ignored", ubx_id);
								break;
							}
							break;
							//other UBX classes go here
							default:
								DEBUGPRINT("ubx: UBX-%s-0x%x ignored", UBX_classnames[ubx_class], ubx_id);
								break;
						}

					}
					else
					{
						ERRORPRINT("ubx: %s-0x%x bad csum: 0x%x/0x%x 0x%x/0x%x ", UBX_classnames[ubx_class], ubx_id, CK_A, PL_CK_A, CK_B, PL_CK_B );
					}
				}
			}
			break;
			default:
				break;
			}
		}
	} catch (std::exception &e) {
		PS_ERROR("gps: thread exception: %s", e.what());
	}
}

//make new data available to the navigator
bool GPS_UBLOX::new_gps_data()
{
	//critical section
	std::unique_lock<std::mutex> lck {gpsDataMutex};

	dateValid		= latest_fix.dateValid;
	timeValid		= latest_fix.timeValid;
	latest_fix.dateValid = latest_fix.timeValid = false;

	update_time 	= latest_update_time;
	fix_time 		= latest_fix_time;

	if (newGPSdata)
	{
		longitude 		= latest_fix.lon;
		latitude 		= latest_fix.lat;
		horizontalAcc 	= latest_fix.hAcc;
		velocityN 		= latest_fix.velN;
		velocityE 		= latest_fix.velE;
		groundSpeed 	= latest_fix.gSpeed;
		headMotion 		= latest_fix.headMot;
		speedAcc 		= latest_fix.sAcc;
		headAcc 		= latest_fix.headAcc;

		positionValid	= latest_fix.positionValid;
		speedValid		= latest_fix.speedValid;
		headingValid	= latest_fix.headingValid;

		newGPSdata = latest_fix.positionValid = latest_fix.speedValid = latest_fix.headingValid = false;

		return true;
	}
	else return false;
}

GPS_UBLOX &the_UBLOX_gps()
{
	static GPS_UBLOX me;
	return me;
}

GPSClass &the_gps()
{
	return the_UBLOX_gps();
}

const char *UBX_classnames[] = {
		"Class 0",
		"NAV",
		"RXM",
		"Class 3",
		"INF",
		"ACK",
		"CFG",
		"Class 7",
		"Class 8",
		"UPD",
		"MON",
		"AID",
		"Class C",
		"TIM",
		"Class E",
		"Class F",
		"ESF",
		"Class 11",
		"Class 12",
		"MGA",
		"Class 14",
		"Class 15",
		"Class 16",
		"Class 17",
		"Class 18",
		"Class 19",
		"Class 1A",
		"Class 1B",
		"Class 1C",
		"Class 1D",
		"Class 1E",
		"Class 1F",
		"Class 20",
		"LOG",
		"Class 22",
		"Class 23",
		"Class 24",
		"Class 25",
		"Class 26",
		"SEC",
		"HNR"};
