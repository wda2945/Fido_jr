
-- common battery check
BT.Priority:new({
	name = 'isBatteryOKish',		-- battery not critical; OK or low
	nodes = {
		'isBatteryOK',
		'isBatteryLow',
		'BatteryFailAlert'
		}
});


BT.Priority:new({
	name = 'Retry.Engage.Forward',
		nodes = {
		'Engage',
		'isMotorFailAbort',
		'isMotorFailTimeout',
		'isMotorResponseTimeout',
		'isMotorFailErrors',
		'isMotorFailBattery',
		  
		BT.Sequence:new({
			nodes = {
				'isMotorFailStall',
				'MoveForwardAlwaysSucceed',
				'Engage'
			}						
		}),

		BT.Sequence:new({
			nodes = {
				'isMotorFailDistance',
				'MoveForwardAlwaysSucceed',
				'Engage'
			}					
		}),					
	}
});

BT.RepeatWhileFail:new({			-- right now, retries forever!
	name = 'Retry.Engage',
	nodes = {
		BT.Priority:new({
			nodes = {
				
				BT.InvertDecorator:new ({
					node = BT.Sequence:new({
						nodes = {
							'isBatteryOKish',
							'isPilotReady'
						}
					})
				});
				
				'Engage',
				'isMotorFailAbort',
				'isMotorFailTimeout',
				'isMotorResponseTimeout',
				'isMotorFailErrors',
				'isMotorFailBattery',
				  
				BT.Sequence:new({
					nodes = {
						'isMotorFailStall',
						'MoveBackwardAlwaysSucceed',
						'Retry.Engage.Forward'
					}						
				}),

				BT.Sequence:new({
					nodes = {
						'isMotorFailDistance',
						'MoveBackwardAlwaysSucceed',
						'Retry.Engage.Forward'
					}					
				}),					

			}
		})
	}
});

--go to waypoint
GoTo = BT:new({
	tree = BT.Sequence:new({
		name = 'Go.To.Waypoint',
		nodes = {
			'isBatteryOKish',
			'isPilotReady',
			'Retry.Engage',
			'isAtGoal'
		}
	})
});


