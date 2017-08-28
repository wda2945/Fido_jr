//
//. Settings.h
//  Fido Jr Euclid
//
//  Created by Martin Lane-Smith on 6/14/14.
//  Copyright (c) 2014 Martin Lane-Smith. All rights reserved.
//

//settingmacro("name", var, min, max, def)

//depth map bounding box
	settingmacro("mMinY",  mMinY, 0, 480, 0)
	settingmacro("mMaxY",  mMaxY, 0, 480, 480)
	settingmacro("mMinX",  mMinX, 0, 640, 0)
	settingmacro("mMaxX",  mMaxX, 0, 640, 640)
	settingmacro("mMinZ",  mMinZ, 0, 10, 0)
	settingmacro("mMaxZ",  mMaxZ, 0, 10, 10)
	settingmacro("mMinBlobSize",  mMinBlobSize, 100, 100000, 40000)

	settingmacro("Far.Depth",  max_far_depth, 0, 10, 3.5)
	settingmacro("Close.Depth",  max_close_depth, 0, 10, 1.3)
	settingmacro("Too.Close.Depth",  max_too_close_depth, 0, 10, 0.7)
