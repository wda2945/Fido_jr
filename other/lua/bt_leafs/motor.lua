--local BT = require('/root/lua/behaviortree/behaviour_tree')

--[[
	Stop,
	Turn,
	TurnLeft,
	TurnRight,
	TurnN,
	TurnS,
	TurnE,
	TurnW,
	TurnLeft90,
	TurnRight90,
	MoveForward,
	MoveBackward,
	MoveForward30,
	MoveBackward30,
	MoveForward60,
	MoveBackward60,
	SetFastSpeed,
	SetMediumSpeed,
	SetLowSpeed,
	EnableFrontCloseStop,
	DisableFrontCloseStop,
	EnableRearCloseStop,
	DisableRearCloseStop,
	EnableFrontFarStop,
	DisableFrontFarStop,
	EnableSystemStop,
	DisableSystemStop,
	isMotorReady,
	isMotorFailAbort,
	isMotorFailDistance,
	isMotorFailHeading,
	isMotorFailProximity,
	isMotorFailBattery,
	isMotorFailErrors,
	isMotorFailInhibit,
	isMotorFailYaw,
	isMotorFailTimeout,
	isMotorFailStall,
	isMotorResponseTimeout,

--]]

BT.Task:new({
  name = 'Stop',
  run = function(self, object)
  	result(MotorAction(motors_Stop))
  end
})

BT.Task:new({
  name = 'Turn',
  run = function(self, object)
  	result(MotorAction(motors_Turn))
  end
})

BT.Task:new({
  name = 'TurnLeft',
  run = function(self, object)
  	result(MotorAction(motors_TurnLeft))
  end
})

BT.Task:new({
  name = 'TurnRight',
  run = function(self, object)
  	result(MotorAction(motors_TurnRight))
  end
})

BT.Task:new({
  name = 'TurnNorth',
  run = function(self, object)
  	result(MotorAction(motors_TurnN))
  end
})

BT.Task:new({
  name = 'TurnSouth',
  run = function(self, object)
  	result(MotorAction(motors_TurnS))
  end
})

BT.Task:new({
  name = 'TurnEast',
  run = function(self, object)
  	result(MotorAction(motors_TurnE))
  end
})

BT.Task:new({
  name = 'TurnWest',
  run = function(self, object)
  	result(MotorAction(motors_TurnW))
  end
})

BT.Task:new({
  name = 'TurnLeft90',
  run = function(self, object)
  	result(MotorAction(motors_TurnLeft90))
  end
})

BT.Task:new({
  name = 'TurnRight90',
  run = function(self, object)
  	result(MotorAction(motors_TurnRight90))
  end
})

BT.Task:new({
  name = 'MoveForward',
  run = function(self, object)
  	result(MotorAction(motors_MoveForward))
  end
})

BT.Task:new({
  name = 'MoveBackward',
  run = function(self, object)
  	result(MotorAction(motors_MoveBackward))
  end
})

BT.Task:new({
  name = 'MoveForward30',
  run = function(self, object)
  	result(MotorAction(motors_MoveForward30))
  end
})

BT.Task:new({
  name = 'MoveBackward30',
  run = function(self, object)
  	result(MotorAction(motors_MoveBackward30))
  end
})

BT.Task:new({
  name = 'MoveForward60',
  run = function(self, object)
  	result(MotorAction(motors_MoveForward60))
  end
})

BT.Task:new({
  name = 'MoveBackward60',
  run = function(self, object)
  	result(MotorAction(motors_MoveBackward60))
  end
})

BT.Task:new({
  name = 'SetFastSpeed',
  run = function(self, object)
  	result(MotorAction(motors_SetFastSpeed))
  end
})

BT.Task:new({
  name = 'SetMediumSpeed',
  run = function(self, object)
  	result(MotorAction(motors_SetMediumSpeed))
  end
})

BT.Task:new({
  name = 'SetLowSpeed',
  run = function(self, object)
  	result(MotorAction(motors_SetLowSpeed))
  end
})
	
BT.Task:new({
  name = 'EnableFrontCloseStop',
  run = function(self, object)
  	result(MotorAction(motors_EnableFrontCloseStop))
  end
})

BT.Task:new({
  name = 'DisableFrontCloseStop',
  run = function(self, object)
  	result(MotorAction(motors_DisableFrontCloseStop))
  end
})

BT.Task:new({
  name = 'EnableRearCloseStop',
  run = function(self, object)
  	result(MotorAction(motors_EnableRearCloseStop))
  end
})

BT.Task:new({
  name = 'DisableRearCloseStop',
  run = function(self, object)
  	result(MotorAction(motors_DisableRearCloseStop))
  end
})

BT.Task:new({
  name = 'EnableFrontFarStop',
  run = function(self, object)
  	result(MotorAction(motors_EnableFrontFarStop))
  end
})

BT.Task:new({
  name = 'DisableFrontFarStop',
  run = function(self, object)
  	result(MotorAction(motors_DisableFrontFarStop))
  end
})

BT.Task:new({
  name = 'EnableSystemStop',
  run = function(self, object)
  	result(MotorAction(motors_EnableSystemStop))
  end
})

BT.Task:new({
  name = 'DisableSystemStop',
  run = function(self, object)
  	result(MotorAction(motors_DisableSystemStop))
  end
})

BT.Task:new({
  name = 'isMotorReady',
  run = function(self, object)
  	result(MotorAction(motors_isMotorReady))
  end
})


BT.Task:new({
  name = 'isMotorFailAbort',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailAbort))
  end
})

BT.Task:new({
  name = 'isMotorFailDistance',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailDistance))
  end
})

BT.Task:new({
  name = 'isMotorFailHeading',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailHeading))
  end
})

BT.Task:new({
  name = 'isMotorFailProximity',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailProximity))
  end
})

BT.Task:new({
  name = 'isMotorFailBattery',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailBattery))
  end
})

BT.Task:new({
  name = 'isMotorFailErrors',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailErrors))
  end
})

BT.Task:new({
  name = 'isMotorFailInhibit',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailInhibit))
  end
})

BT.Task:new({
  name = 'isMotorFailYaw',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailYaw))
  end
})

BT.Task:new({
  name = 'isMotorFailTimeout',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailTimeout))
  end
})

BT.Task:new({
  name = 'isMotorFailStall',
  run = function(self, object)
  	result(MotorAction(motors_isMotorFailStall))
  end
})

BT.Task:new({
  name = 'isMotorResponseTimeout',
  run = function(self, object)
  	result(MotorAction(motors_isMotorResponseTimeout))
  end
})
