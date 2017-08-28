

ReloadScripts = BT:new({
  tree = BT.Sequence:new({
		nodes = {
			'ReloadScripts',
			}
		})
});

SystemPoweroff = BT:new({
  tree = BT.Sequence:new({
		nodes = {
			'SystemPoweroff',
			}
		})
});

SystemReboot = BT:new({
  tree = BT.Sequence:new({
		nodes = {
			'SystemReboot',
			}
		})
});

SystemSetResting = BT:new({
  tree = BT.Sequence:new({
		nodes = {
			'SystemSetResting',
			}
		})
});

SystemSetActive = BT:new({
  tree = BT.Sequence:new({
		nodes = {
			'SystemSetActive',
			}
		})
});

AssumeNorth = BT:new({
  tree = BT.Sequence:new({
		nodes = {
			'AssumeN',
			}
		})
});

ActivityList[NextActivity] =  'ReloadScripts'
NextActivity = NextActivity + 1

ActivityList[NextActivity] =  'SystemPoweroff'
NextActivity = NextActivity + 1

ActivityList[NextActivity] =  'SystemReboot'
NextActivity = NextActivity + 1

ActivityList[NextActivity] =  'AssumeNorth'
NextActivity = NextActivity + 1

