--[[
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
]]

BT.Task:new({
  name = 'SaveSettings',
  run = function(self, object)
  	result(SystemAction(system_SaveSettings))
  end
})

BT.Task:new({
  name = 'LoadSettings',
  run = function(self, object)
  	result(SystemAction(system_LoadSettings))
  end
})

BT.Task:new({
  name = 'ResetSavedSettings',
  run = function(self, object)
  	result(SystemAction(system_ResetSavedSettings))
  end
})

BT.Task:new({
  name = 'SystemReboot',
  run = function(self, object)
  	result(SystemAction(system_SystemReboot))
  end
})

BT.Task:new({
  name = 'SystemPowerCycle',
  run = function(self, object)
  	result(SystemAction(system_SystemPowerCycle))
  end
})

BT.Task:new({
  name = 'SystemPoweroff',
  run = function(self, object)
  	result(SystemAction(system_SystemPoweroff))
  end
})

BT.Task:new({
  name = 'SystemSleep',
  run = function(self, object)
  	result(SystemAction(system_SystemSleep))
  end
})

BT.Task:new({
  name = 'SystemSetResting',
  run = function(self, object)
  	result(SystemAction(system_SystemSetResting))
  end
})

BT.Task:new({
  name = 'SystemSetActive',
  run = function(self, object)
  	result(SystemAction(system_SystemSetActive))
  end
})

BT.Task:new({
  name = 'ReloadScripts',
  run = function(self, object)
  	result(SystemAction(system_ReloadScripts))
  end
})

BT.Task:new({
  name = 'isBatteryCritical',
  run = function(self, object)
  	result(SystemAction(system_isBatteryCritical))
  end
})

BT.Task:new({
  name = 'isBatteryLow',
  run = function(self, object)
  	result(SystemAction(system_isBatteryLow))
  end
})

BT.Task:new({
  name = 'isBatteryOK',
  run = function(self, object)
  	result(SystemAction(system_isBatteryOK))
  end
})