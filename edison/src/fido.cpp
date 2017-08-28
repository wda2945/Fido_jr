//
//  hexapod.cpp
//
//  Created by Martin Lane-Smith on 7/2/14.
//  Copyright (c) 2014 Martin Lane-Smith. All rights reserved.
//
// Starts all robot processes

#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <ctype.h>
#include <dirent.h>
#include <libgen.h>
#include <scanner/proximity.hpp>
#include <termios.h>
#include <string>
#include <ctime>
#include <time.h>

#include "robot.h"
#include "software_profile.h"
#include "mraa.h"

#include "lua.hpp"

#include "ps_common.h"

#include "common/ps_root_class.hpp"
#include "pubsub/ps_pubsub_class.hpp"
#include "serial/linux_serial/ps_serial_linux.hpp"
#include "serial/socket/ps_socket_server.hpp"
#include "packet/serial_packet/ps_packet_serial_linux.hpp"
#include "packet/xbee_packet/ps_packet_xbee_linux.hpp"
#include "packet/xbee_packet/ps_packet_xbee_class.hpp"
#include "transport/ps_transport_linux.hpp"
#include "syslog/ps_syslog_linux.hpp"
#include "network/ps_network.hpp"

#include "autopilot/autopilot.hpp"
#include "behavior/behavior.hpp"
#include "navigator/navigator.hpp"
#include "responder/responder.hpp"

#include "main_debug.h"

using namespace std;

FILE *mainDebugFile;

#define PROCESS_NAME "FidoJr"

bool initComplete = false;

int InitSystemCallbacks();

void KillAllOthers(string name);
void fatal_error_signal (int sig);
void SIGPIPE_signal (int sig){}

class PIC_Observer : public ps_root_class
{
public:
     void message_handler(ps_packet_source_t packet_source,
                                 ps_packet_type_t   packet_type,
                                 const void *msg, int length) override {};
    //observer callbacks
     void process_observed_data(ps_root_class *src, const void *msg, int length) override {};
     void process_observed_event(ps_root_class *src, int event) override;
};

static PIC_Observer pic_observer;

void PIC_Observer::process_observed_event(ps_root_class *src, int event)
{
	switch (event)
	{
	case PS_TRANSPORT_ONLINE:
		ps_set_condition(PIC_ONLINE);
		break;
	case PS_TRANSPORT_OFFLINE:
		ps_cancel_condition(PIC_ONLINE);
		break;
	default:
		break;
	}
}

int main()
{
	string initFail = "";	//fail flag

	//start the log
    the_logger();

	//set up this logging
	mainDebugFile = fopen_logfile("main");

	DEBUGPRINT("Compiled %s %s", __DATE__, __TIME__);

    DEBUGPRINT("MAX_USER_MESSAGE = %d", MAX_USER_MESSAGE);
    DEBUGPRINT("SYSLOG_PACKET = %d", SYSLOG_PACKET_SIZE);
    DEBUGPRINT("REGISTRY_PACKET = %d", REGISTRY_PACKET_SIZE);
    DEBUGPRINT("MAX_TRANSPORT_PACKET = %d", MAX_TRANSPORT_PACKET);

	//kill any running services
	KillAllOthers(PROCESS_NAME);

	sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGPIPE);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

	//set SIGPIPE handler
	const struct sigaction sa {SIGPIPE_signal, 0, 0};
	sigaction(SIGPIPE, &sa, NULL);

	mraa_result_t mraa_reply = mraa_init();
	if (mraa_reply != MRAA_SUCCESS)
	{
		ERRORPRINT("mraa: init error");
		mraa_result_print(mraa_reply);
	}

	//plumbing
	DEBUGPRINT("Registry init");
	//initialize the registry
	ps_register_event_names(eventNames, EVENT_COUNT);
	ps_register_condition_names(conditionNames, EDISON_CONDITIONS_COUNT);
	ps_register_topic_names(psTopicNames, PS_TOPIC_COUNT);

	//start the socket server
	DEBUGPRINT("fido jr socket server init");
	//socket server
	ps_socket_server *socket_server = new ps_socket_server(FIDO_JR_LISTEN_PORT, PING_PORT_NUMBER);
	if (socket_server)
	{
		//socket packet layer
		ps_packet_serial_linux *socket_pkt = new ps_packet_serial_linux("socket_server", socket_server);
		if (socket_pkt)
		{
			//socket transport layer
			ps_transport_linux *socket_transport = new ps_transport_linux(
					socket_pkt);
			if (socket_transport)
			{
//			socket_transport->transport_source = SRC_IOSAPP;
			
			socket_transport->source_filter[SRC_IOSAPP] = false;                      
            socket_transport->source_filter[SRC_FIDOJR_PIC] = true;
            socket_transport->source_filter[SRC_FIDOJR_EDI] = true;
                    
			//add to pubsub network
			the_network().add_transport_to_network(socket_transport);

			the_broker().subscribe_to_topic(RESPONSE_TOPIC ,socket_transport);
			the_broker().subscribe_to_topic(SYS_REPORT_TOPIC ,socket_transport);
			the_broker().subscribe_to_topic(NAV_TOPIC ,socket_transport);

			the_broker().subscribe_to_packet(SYSLOG_PACKET ,socket_transport);
			the_broker().subscribe_to_packet(REGISTRY_UPDATE_PACKET ,socket_transport);
			the_broker().subscribe_to_packet(REGISTRY_SYNC_PACKET ,socket_transport);
			the_broker().subscribe_to_packet(CONDITIONS_PACKET ,socket_transport);
			}
			else
			{
				ERRORPRINT("main: new socket transport layer fail");
				ps_set_condition(EDISON_INIT_ERROR);
			}
		}
		else
		{
			ERRORPRINT("main: new socket pkt layer fail");
			ps_set_condition(EDISON_INIT_ERROR);
		}
	}
	else
	{
		ERRORPRINT("main: new socket server fail");
		ps_set_condition(EDISON_INIT_ERROR);
	}
	//start PIC transport
	DEBUGPRINT("PIC transport init");

	//wrap the serial port
	ps_serial_linux *pic_serial = new ps_serial_linux("pic_serial", UART_PATH, B115200);
	if (pic_serial)
	{
		//add packet layer
		ps_packet_serial_linux *pic_packet = new ps_packet_serial_linux("pic_serial", pic_serial);
		if (pic_packet)
		{
			//add transport layer
			ps_transport_linux *pic_transport = new ps_transport_linux(pic_packet);
			if (pic_transport)
			{
				pic_transport->add_event_observer(&pic_observer);
			
				pic_transport->source_filter[SRC_IOSAPP] = false;                      
            	pic_transport->source_filter[SRC_FIDOJR_PIC] = false;
            	pic_transport->source_filter[SRC_FIDOJR_EDI] = true;
            
				//add to network
				the_network().add_transport_to_network(pic_transport);

				the_broker().subscribe_to_topic(RESPONSE_TOPIC ,pic_transport);
				the_broker().subscribe_to_topic(SYS_REPORT_TOPIC ,pic_transport);
				the_broker().subscribe_to_topic(NAV_TOPIC ,pic_transport);
				the_broker().subscribe_to_topic(MOT_ACTION_TOPIC, pic_transport);
				the_broker().subscribe_to_topic(SOUND_TOPIC, pic_transport);
				the_broker().subscribe_to_topic(TICK_TOPIC, pic_transport);

				//	    the_broker().subscribe_to_topic(TICK_1S_TOPIC, pic_transport);

				the_broker().subscribe_to_packet(SYSLOG_PACKET ,pic_transport);
				the_broker().subscribe_to_packet(REGISTRY_UPDATE_PACKET ,pic_transport);
				the_broker().subscribe_to_packet(REGISTRY_SYNC_PACKET ,pic_transport);
				the_broker().subscribe_to_packet(CONDITIONS_PACKET ,pic_transport);
			}
			else
			{
				ERRORPRINT("main: new PIC transport layer fail");
				ps_set_condition(EDISON_INIT_ERROR);
			}
		}
		else
		{
			ERRORPRINT("main: new PIC packet layer fail");
			ps_set_condition(EDISON_INIT_ERROR);
		}
	}
	else
	{
		ERRORPRINT("main: new PIC serial layer fail");
		ps_set_condition(EDISON_INIT_ERROR);
	}

	//init subsystems

	try
	{
		//load waypoint DB
		the_waypoints().LoadWaypointDatabase(WAYPOINT_FILE_PATH);

		//responder
		initFail = "responder";
		the_responder_instance();
		DEBUGPRINT("Responder init OK");

		//start navigator threads
		initFail = "navigator";
		the_navigator_instance();
		DEBUGPRINT("Navigator init OK");

		//start autopilot threads
		initFail = "autopilot";
		the_autopilot_instance();
		DEBUGPRINT("Autopilot init OK");

		//start behavior tree threads
		initFail = "behavior";
		the_behaviors_instance();
		DEBUGPRINT("Behaviors init OK");
		
		InitSystemCallbacks();
	}
	catch (string &s)
	{
		ps_set_condition(EDISON_INIT_ERROR);
		LogError("Fido_jr Init Fail '%s' in %s", s.c_str(), initFail.c_str());
		sleep(10);
		return -1;
	}

	while (!initComplete)
	{
		if (!ps_test_condition(SOURCE, PIC_ONLINE))
		{
			DEBUGPRINT("main: Waiting for PIC I/F");
		}
		else if (!ps_test_condition(SOURCE, IMU_DATA) && waitIMU)
		{
			DEBUGPRINT("main: Waiting for IMU");
			speak("I am waiting for the I M U.");
		}
		else if (ps_test_condition(SOURCE, GPSTTY_NF_ERROR))
		{
			speak("G P S serial port cannot be found. Re-start the processor.");
		}
		else if (!ps_test_condition(SOURCE, GPS_FIX) && waitGPS)
		{
			DEBUGPRINT("main: Waiting for GPS Fix");
			speak("I am waiting for a G P S fix.");
		}
		else
		{
			initComplete = true;
		}

		std::this_thread::sleep_for(std::chrono::seconds(5));
	}

	ps_set_condition(SYS_READY);
	LogRoutine("Init complete");
	speak("The system is ready.");

	if (getppid() == 1)
	{
		//child of init/systemd

		//close stdio
		fclose(stdout);
		fclose(stderr);
		stdout = fopen("/dev/null", "w");
		stderr = fopen("/dev/null", "w");
	}

	signal(SIGILL, fatal_error_signal);
	signal(SIGABRT, fatal_error_signal);
	signal(SIGIOT, fatal_error_signal);
	signal(SIGBUS, fatal_error_signal);
	signal(SIGFPE, fatal_error_signal);
	signal(SIGSEGV, fatal_error_signal);
	signal(SIGTERM, fatal_error_signal);
	signal(SIGCHLD, fatal_error_signal);
	signal(SIGSYS, fatal_error_signal);
	signal(SIGCHLD, fatal_error_signal);

	time_t now;
	struct tm *last_minute = std::localtime(&now);
	int last_min = last_minute->tm_min;

	psMessage_t msg;
	psInitPublish(msg, TICK);

	while(1)
	{
		int i;
		for (i=0; i<30; i++)
		{
			sleep(1);
			now = time(NULL);
			struct tm *current = std::localtime(&now);
			if (current->tm_sec == 0 && current->tm_min != last_min)
			{
				snprintf(msg.tickPayload.text, PS_TICK_TEXT, "%2d/%2d %02d:%02d",
						current->tm_mon, current->tm_mday, current->tm_hour, current->tm_min);
				DEBUGPRINT("tick: %s", msg.tickPayload.text);
				NewBrokerMessage(msg);

				last_min = current->tm_min;
			}
		}
		if (commStats) ps_debug_transport_stats();
	}
	return 0;
}

void speak(const char *phrase) {
    psMessage_t msg;
    psInitPublish(msg, SPEAK);
    strncpy(msg.speechPayload.text, phrase, MAX_SPEECH);
    msg.speechPayload.text[MAX_SPEECH - 1] = '\0';
    RouteMessage(msg);
}

//other signals
volatile sig_atomic_t fatal_error_in_progress = 0;
void fatal_error_signal (int sig)
{
	/* Since this handler is established for more than one kind of signal, it might still get invoked recursively by delivery of some other kind of signal. Use a static variable to keep track of that. */
	if (fatal_error_in_progress) raise (sig);

	fatal_error_in_progress = 1;

	LogError("Signal %i raised", sig);
	sleep(1);	//let there be printing

	/* Now re-raise the signal. We reactivate the signal�s default handling, which is to terminate the process. We could just call exit or abort,
but re-raising the signal sets the return status
from the process correctly. */
	signal (sig, SIG_DFL);
	raise (sig);
}

//helper functions to find any existing processes of a given name

/* checks if the string is purely an integer
 * we can do it with `strtol' also
 */
int check_if_number (char *str)
{
  int i;
  for (i=0; str[i] != '\0'; i++)
  {
    if (!isdigit (str[i]))
    {
      return 0;
    }
  }
  return 1;
}

#define MAX_BUF 1024
#define PID_LIST_BLOCK 32

//returns a list of up to 32 pids of processes matching the provided name
int *pidof (string pname)
{
  DIR *dirp;
  FILE *fp;
  struct dirent *entry;
  int *pidlist, pidlist_index = 0, pidlist_realloc_count = 1;
  char path[MAX_BUF], read_buf[MAX_BUF];

  dirp = opendir ("/proc/");
  if (dirp == NULL)
  {
    perror ("Fail");
    return NULL;
  }

  pidlist = (int*) malloc (sizeof (int) * PID_LIST_BLOCK);
  if (pidlist == NULL)
  {
    return NULL;
  }

  while ((entry = readdir (dirp)) != NULL)
  {
    if (check_if_number (entry->d_name))
    {
      strcpy (path, "/proc/");
      strcat (path, entry->d_name);
      strcat (path, "/comm");

      /* A file may not exist, it may have been removed.
       * due to termination of the process. Actually we need to
       * make sure the error is actually file does not exist to
       * be accurate.
       */
      fp = fopen (path, "r");
      if (fp != NULL)
      {
        fscanf (fp, "%s", read_buf);
        if (pname.compare(read_buf) == 0)
        {
          /* add to list and expand list if needed */
          pidlist[pidlist_index++] = atoi (entry->d_name);
          if (pidlist_index == PID_LIST_BLOCK * pidlist_realloc_count)
          {
            pidlist_realloc_count++;
            pidlist = (int*) realloc (pidlist, sizeof (int) * PID_LIST_BLOCK * pidlist_realloc_count); //Error check
            if (pidlist == NULL)
            {
              return NULL;
            }
          }
        }
        fclose (fp);
      }
    }
  }

  closedir (dirp);
  pidlist[pidlist_index] = -1; /* indicates end of list */
  return pidlist;
}

void KillAllOthers(string name)
{
	//kill any others of this name
	int *pidlist = pidof(name);	//list of pids
	int *pids = pidlist;
	//kill each pid in list (except me)
	while (*pids != -1) {
		if (*pids != getpid())	//don't kill me
		{
			kill(*pids, SIGTERM);
			DEBUGPRINT("Killed pid %i (%s)", *pids, name.c_str());
		}
		pids++;
	}
	free(pidlist);
}


