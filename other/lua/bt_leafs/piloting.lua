--local BT = require('/root/lua/behaviortree/behaviour_tree')

--[[
	isPilotReady,
	WaitUntilPilotReady,
	ComputeHomePosition,
	ComputeRandomExplorePosition,
	ComputeRandomClosePosition,
	Orient,
	Engage,
	PilotReset,
	isAtGoal
]]

BT.Task:new({
  name = 'isPilotReady',
  run = function(self, object)
  	result(PilotAction(Pilot_isPilotReady))
  end
})

BT.Task:new({
  name = 'WaitUntilPilotReady',
  run = function(self, object)
  	result(PilotAction(Pilot_WaitUntilPilotReady))
  end
})

BT.Task:new({
  name = 'ComputeHomePosition',
  run = function(self, object)
  	result(PilotAction(Pilot_ComputeHomePosition))
  end
})

BT.Task:new({
  name = 'ComputeRandomExplorePosition',
  run = function(self, object)
  	result(PilotAction(Pilot_ComputeRandomExplorePosition))
  end
})

BT.Task:new({
  name = 'ComputeRandomClosePosition',
  run = function(self, object)
  	result(PilotAction(Pilot_ComputeRandomClosePosition))
  end
})

BT.Task:new({
  name = 'Orient',
  run = function(self, object)
  	result(PilotAction(Pilot_Orient))
  end
})

BT.Task:new({
  name = 'Engage',
  run = function(self, object)
  	result(PilotAction(Pilot_Engage))
  end
})

BT.Task:new({
  name = 'PilotReset',
  run = function(self, object)
  	result(PilotAction(Pilot_PilotReset))
  end
})

BT.Task:new({
  name = 'isAtGoal',
  run = function(self, object)
  	result(PilotAction(Pilot_isAtGoal))
  end
})