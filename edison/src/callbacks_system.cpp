/*
 * callbacks_system.cpp
 *
 *  Created on: Aug 10, 2014
 *      Author: martin
 */
// BT Leaf callbacks for lua
// System Callbacks

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include <unistd.h>
#include <linux/reboot.h>
#include <sys/reboot.h>

#include "ps.h"
#include "lua.hpp"

#include "robot.h"

#include "main_debug.h"
#include "BehaviorClass.hpp"

//actual leaf node
static int SystemAction(lua_State *L);

int SaveSettingsAndOptions(lua_State *L, bool useDefault);
int LoadSettingsAndOptions(lua_State *L);

typedef enum {
	SaveSettings,
	LoadSettings,
	ResetSavedSettings,
	SystemReboot,
	SystemPowerCycle,	//reboot plus power cycle
    SystemPoweroff,		//total power off
	SystemSleep,		//Edison power off
    SystemSetResting,	//motors & sensors off
    SystemSetActive,	//full power
    ReloadScripts,
	isBatteryCritical,
	isBatteryLow,
	isBatteryOK,
	SYSTEM_ACTION_COUNT
} SystemAction_enum;

static const char *systemActionList[] = {
		"system_SaveSettings",
		"system_LoadSettings",
		"system_ResetSavedSettings",
		"system_SystemReboot",
		"system_SystemPowerCycle",
	    "system_SystemPoweroff",
		"system_SystemSleep",
	    "system_SystemSetResting",
	    "system_SystemSetActive",
	    "system_ReloadScripts",
		"system_isBatteryCritical",
		"system_isBatteryLow",
		"system_isBatteryOK"
};

//don't use
void dummy_power_callback(const char *domain, const char *name, const void *args){}

int InitSystemCallbacks()
{
	//initialize lua callbacks
	the_behaviors().register_lua_callback("SystemAction", &SystemAction);

	int i;
	for (i=0; i< SYSTEM_ACTION_COUNT; i++)
	{
		the_behaviors().register_lua_global(systemActionList[i], i);
	}

	ps_registry_set_observer("Power", "any", dummy_power_callback, nullptr);		//make sure registry knows about "Power"

	return 0;
}

static int SystemAction(lua_State *L)
{
	psMessage_t msg;

	SystemAction_enum actionCode 	= (SystemAction_enum) lua_tointeger(L, 1);

	the_behaviors().lastLuaCall = systemActionList[actionCode];

	DEBUGPRINT("System Action: %s", systemActionList[actionCode]);

	switch (actionCode)
	{
	case SaveSettings:
		return SaveSettingsAndOptions(L, false);
		break;
	case LoadSettings:
		return LoadSettingsAndOptions(L);
		break;
	case ResetSavedSettings:
		return SaveSettingsAndOptions(L, true);
		break;
	case SystemReboot:
		reboot(RB_AUTOBOOT);
		return running(L);
		break;
	case SystemPowerCycle:
		ps_registry_set_text("Power", "Level", POWER_CYCLE);
		sleep(1);
		reboot(RB_POWER_OFF);
		return running(L);
	case SystemPoweroff:
		ps_registry_set_text("Power", "Level", POWER_DOWN);
		sleep(1);
		reboot(RB_POWER_OFF);
		return running(L);
		break;
	case SystemSleep:
		ps_registry_set_text("Power", "Level", POWER_SLEEP);
		sleep(1);
		reboot(RB_POWER_OFF);
		return running(L);
		break;
	case SystemSetResting:
		ps_registry_set_text("Power", "Level", POWER_AWAKE);
		return success(L);
		break;
	case SystemSetActive:
		ps_registry_set_text("Power", "Level", POWER_FULL);
		return success(L);
		break;
	case ReloadScripts:
		psInitPublish(msg, RELOAD);
		RouteMessage(msg);
		return success(L);
		break;
	case isBatteryCritical:
		if (ps_test_condition(SRC_FIDOJR_PIC, PIC::BATTERY_CRITICAL))
		{
			return success(L);
		}
		else
		{
			return fail(L);
		}
		break;
	case isBatteryLow:
		if (ps_test_condition(SRC_FIDOJR_PIC, PIC::BATTERY_LOW))
		{
			return success(L);
		}
		else
		{
			return fail(L);
		}
		break;
	case isBatteryOK:
		if (ps_test_condition(SRC_FIDOJR_PIC, PIC::BATTERY_CRITICAL)
				|| ps_test_condition(SRC_FIDOJR_PIC, PIC::BATTERY_LOW))
		{
			return fail(L);
		}
		else
		{
			return success(L);
		}
		break;
	default:
		ERRORPRINT("Sys code: %i", actionCode);
		return fail(L);
		break;
	}
}

int SaveSettingsAndOptions(lua_State *L, bool useDefault)
{
	bool actionFail = false;

	FILE *fp = fopen(SAVED_SETTINGS_FILE, "w");
	if (fp)
	{
		//Settings
#define settingmacro(name, var, min, max, def) if (fprintf(fp, "'%s';%f\n", name, (useDefault ? def : var)) < 0) actionFail = true;
#include "settings.h"
#undef settingmacro

		fclose(fp);

		fp = fopen(SAVED_OPTIONS_FILE, "w");
		if (fp)
		{
			//options
#define optionmacro(name, var, min, max, def) if (fprintf(fp, "'%s';%i\n", name, (useDefault ? def : var)) < 0) actionFail = true;
#include "options.h"
#undef optionmacro

			fclose(fp);

			if (actionFail) {
				return fail(L);
			}
			else
			{
				LogRoutine("Settings and Options Saved");
				return success(L);
			}
		}
		else
		{
			LogError("Options.txt - %s", strerror(errno));
			return fail(L);
		}
	}
	else
	{
		LogError("Options.txt - %s", strerror(errno));
		return  fail(L);
	}
}

int LoadSettingsAndOptions(lua_State *L)
{
	char name[80];
	float setting;
	int option;
	int result;

	FILE *fp = fopen(SAVED_SETTINGS_FILE, "r");
	if (fp)
	{
		do {
			result = fscanf(fp, "'%79s';%f\n", name, &setting);
			if (result  == 2)
			{
				//Settings
#define settingmacro(n, var, min, max, def) if (strncmp(name, n, 80) == 0) var = setting;
#include "settings.h"
#undef settingmacro
			}
			else if (result < 0)
			{
				LogError("Settings read: %s", strerror(errno));
				return fail(L);
			}
		} while (result > 0);

		fclose(fp);

		fp = fopen(SAVED_OPTIONS_FILE, "r");
		if (fp)
		{
			do {
				result = fscanf(fp, "'%79s';%i\n", name, &option);
				if (result  == 2)
				{
					//options
#define optionmacro(n, var, min, max, def) if (strncmp(name, n, 80) == 0) var = option;
#include "options.h"
#undef optionmacro
				}
				else if (result < 0)
				{
					LogError("Options read: %s", strerror(errno));
					return fail(L);
				}
			} while (result > 0);

			fclose(fp);

			LogRoutine("Settings and Options Saved");
			return success(L);
		}
		else
		{
			LogError("Settings.txt: %s", strerror(errno));
			return  fail(L);
		}
	}
	else
	{
		LogError("Settigns.txt: %s", strerror(errno));
		return  fail(L);
	}
}
