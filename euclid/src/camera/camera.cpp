/* 
 * File:   camera.cpp
 * Author: martin
 *
 * Streams RealSense camera and checks for obstacles
 *
 */
 
#include "ps.h"
#include "main_debug.h"
#include "robot.h"
#include "camera.hpp"

fido_camera &the_camera()
{
	static fido_camera me;
	return me;
}

fido_camera::fido_camera()
{
    DEBUGPRINT("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return;

    dev = ctx.get_device(0);

    DEBUGPRINT("\nUsing device 0, an %s\n", dev->get_name());
    DEBUGPRINT("    Serial number: %s\n", dev->get_serial());
    DEBUGPRINT("    Firmware version: %s\n", dev->get_firmware_version());

    fido_camera_thread = new std::thread([this](){fido_camera_thread_method();});
}

void fido_camera::fido_camera_thread_method() try
{
    // Configure all streams to run at VGA resolution at 60 frames per second

    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
//    dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
//    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);

//    try { dev->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 60); }
//    catch(...) { DEBUGPRINT("Device does not provide infrared2 stream.\n"); }

    dev->start();

    // Determine depth value corresponding to one meter
    one_meter = static_cast<uint16_t>(1.0f / dev->get_depth_scale());

    while(1)
    {
        // Wait for new frame data
        dev->wait_for_frames();

        // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
        const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev->get_frame_data(rs::stream::depth));

        process(depth_frame);
    }
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    ERRORPRINT("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    ERRORPRINT("    %s\n", e.what());
    return;
}

bool fido_camera::enoughPointsDetected(const uint16_t*  cloud, float targetZ, bool freespace)
{

	//Number of points observed
	unsigned int n = 0;
	//Iterate through all the points in the region and find the average of the position
    for(int y=0; y<480; ++y)
    {
        for(int x=0; x<640; ++x)
        {
            int depth = *depth_frame++;
            float z = (float) depth / one_meter;

			//Test to ensure the point is within the aceptable box.
			if (z > mMinZ && z < targetZ && -y > mMinY && -y < mMaxY && (freespace || (x < mMaxX && x > mMinX)))
			{
				n++;
			}
        }
    }

	//If there are enough points
	return (n > mMinBlobSize);
}

// Check what direction to turn
int fido_camera::calcDirection(const uint16_t*  cloud, float targetZ)
{
	float leftDir = 0;
	float rightDir = 0;

	//Iterate through all the points in the region and find the average of the position
    for(int y=0; y<480; ++y)
    {
        for(int x=0; x<640; ++x)
        {
            int depth = *depth_frame++;
            float z = (float) depth / one_meter;

			//Test to ensure the point is within the aceptable box.
			if (z > mMinZ && z < targetZ && -y > mMinY && -y < mMaxY && x < mMaxX && x > mMinX)
			{
				if (x < 320)
					leftDir++;
				else
					rightDir++;
			}
        }
    }

	if (leftDir > rightDir)
		return -1;
	else
		return 1;
}


void fido_camera::process(const uint16_t*  cloud)
{
	bool obstacleDetected = false;
	bool shouldReverse = false;
	bool closeObstacleDetected = false;

	if (enoughPointsDetected(cloud, max_far_depth, true)) //we detect an edge in front
	{
		if (enoughPointsDetected(cloud, max_close_depth, false)) //too close. it is an obstacle
		{
			obstacleDetected = true;
			//closeObstacleDetected = true;
			if (enoughPointsDetected(cloud, max_too_close_depth, false)) //very close. it is an obstacle
			{
				closeObstacleDetected = true;
			}
		}
	}
	else
	{	//no edge. consider it as if it is an obstacle. It might be becuase we are too close.
		shouldReverse = true;
	}

	if (obstacleDetected)
	{
		if (closeObstacleDetected)
		{
			if (calcDirection(cloud, 1.0) == -1)
			{
				ps_set_condition(FRONT_LEFT_PROXIMITY);
				ps_set_condition(FRONT_CENTER_PROXIMITY);
				ps_cancel_condition(FRONT_RIGHT_PROXIMITY);
			}
			else
			{
				ps_set_condition(FRONT_RIGHT_PROXIMITY);
				ps_set_condition(FRONT_CENTER_PROXIMITY);
				ps_cancel_condition(FRONT_LEFT_PROXIMITY);
			}
			ps_cancel_condition(FRONT_LEFT_FAR_PROXIMITY);
			ps_cancel_condition(FRONT_RIGHT_FAR_PROXIMITY);
		}
		else
		{
			if (calcDirection(cloud, 1.0) == -1)
			{
				ps_set_condition(FRONT_LEFT_FAR_PROXIMITY);
			}
			else
			{
				ps_set_condition(FRONT_RIGHT_FAR_PROXIMITY);
			}
			ps_cancel_condition(FRONT_LEFT_PROXIMITY);
			ps_cancel_condition(FRONT_RIGHT_PROXIMITY);
			ps_cancel_condition(FRONT_CENTER_PROXIMITY);
		}
	}
	else
	{
		if (shouldReverse)
		{
			ps_set_condition(FRONT_LEFT_PROXIMITY);
			ps_set_condition(FRONT_RIGHT_PROXIMITY);
			ps_set_condition(FRONT_CENTER_PROXIMITY);
		}
		else
		{
			ps_cancel_condition(FRONT_LEFT_PROXIMITY);
			ps_cancel_condition(FRONT_RIGHT_PROXIMITY);
			ps_cancel_condition(FRONT_CENTER_PROXIMITY);
		}
		ps_cancel_condition(FRONT_LEFT_FAR_PROXIMITY);
		ps_cancel_condition(FRONT_RIGHT_FAR_PROXIMITY);
	}

}
