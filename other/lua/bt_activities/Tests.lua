

TurnLeft90 = BT:new({
  name = 'Test.Turn.Left.90',
  tree = BT.Sequence:new({
	name = 'Test.Seq',
		nodes = {
			'DisableFrontCloseStop',
			'DisableRearCloseStop',
			'TurnLeft90',
			}
		})
});

ActivityList[NextActivity] =  'TurnLeft90'
NextActivity = NextActivity + 1

TurnRight90 = BT:new({
  name = 'Test.Turn.Right.90',
  tree = BT.Sequence:new({
	name = 'Test.Seq',
		nodes = {
			'DisableFrontCloseStop',
			'DisableRearCloseStop',
			'TurnRight90',
			}
		})
});

ActivityList[NextActivity] =  'TurnRight90'
NextActivity = NextActivity + 1


Forward60 = BT:new({
  name = 'Test.Forward.60',
  tree = BT.Sequence:new({
	name = 'Test.Seq',
		nodes = {
			'DisableFrontCloseStop',
			'DisableRearCloseStop',
			'MoveForward60',
			}
		})
});

ActivityList[NextActivity] =  'Forward60'
NextActivity = NextActivity + 1

Backward60 = BT:new({
  name = 'Test.Backward.60',
  tree = BT.Sequence:new({
	name = 'Test.Seq',
		nodes = {
			'DisableFrontCloseStop',
			'DisableRearCloseStop',
			'MoveBackward60',
			}
		})
});

ActivityList[NextActivity] =  'Backward60'
NextActivity = NextActivity + 1



