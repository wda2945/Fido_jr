--local BT = require('/root/lua/behaviortree/behaviour_tree')

--[[
	AssumeHome,
	AssumeN,
	OrientationValid,
	OrientationDR,
	OrientationAbsolute,
	LocationValid,
	LocationDGPS,
	Location10cm,
]]

BT.Task:new({
  name = 'AssumeHome',
  run = function(self, object)
  	result(NavigatorAction(Navigator_AssumeHome))
  end
})

BT.Task:new({
  name = 'AssumeN',
  run = function(self, object)
  	result(NavigatorAction(Navigator_AssumeN))
  end
})

BT.Task:new({
  name = 'OrientationValid',
  run = function(self, object)
  	result(NavigatorAction(Navigator_OrientationValid))
  end
})

BT.Task:new({
  name = 'OrientationDR',
  run = function(self, object)
  	result(NavigatorAction(Navigator_OrientationDR))
  end
})

BT.Task:new({
  name = 'OrientationAbsolute',
  run = function(self, object)
  	result(NavigatorAction(Navigator_OrientationAbsolute))
  end
})

BT.Task:new({
  name = 'LocationDGPS',
  run = function(self, object)
  	result(NavigatorAction(Navigator_LocationDGPS))
  end
})

BT.Task:new({
  name = 'Location10cm',
  run = function(self, object)
  	result(NavigatorAction(Navigator_Location10cm))
  end
})
