--[[
	isFrontLeftProximity,
	isFrontProximity,
	isFrontRightProximity,
	isRearLeftProximity,
	isRearProximity,
	isRearRightProximity,
	isLeftProximity,
	isRightProximity,
	isFrontLeftFarProximity,
	isFrontFarProximity,
	isFrontRightFarProximity,
]]
	
BT.Task:new({
  name = 'isFrontLeftProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isFrontLeftProximity))
  end
})
	
BT.Task:new({
  name = 'isFrontProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isFrontProximity))
  end
})
	
BT.Task:new({
  name = 'isFrontRightProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isFrontRightProximity))
  end
})
	
BT.Task:new({
  name = 'isRearLeftProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isRearLeftProximity))
  end
})
	
BT.Task:new({
  name = 'isRearProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isRearProximity))
  end
})
	
BT.Task:new({
  name = 'isRearRightProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isRearRightProximity))
  end
})
	
BT.Task:new({
  name = 'isLeftProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isLeftProximity))
  end
})
	
BT.Task:new({
  name = 'isRightProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isRightProximity))
  end
})
	
BT.Task:new({
  name = 'isFrontLeftFarProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isFrontLeftFarProximity))
  end
})
	
BT.Task:new({
  name = 'isFrontFarProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isFrontFarProximity))
  end
})
	
BT.Task:new({
  name = 'isFrontRightFarProximity',
  run = function(self, object)
  	result(ProximityAction(Proximity_isFrontRightFarProximity))
  end
})
	