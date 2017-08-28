//
//  Camera.hpp
//  FidoJr Euclid
//
//  Created by Martin Lane-Smith on 8/20/17.
//  Copyright (c) 2017 Martin Lane-Smith. All rights reserved.
//
#include <thread>
// The librealsense C++ header file
#include <librealsense/rs.hpp>

class fido_camera
{
public:


private:
	fido_camera();
	~fido_camera() {}

	std::thread *fido_camera_thread;
	void fido_camera_thread_method();

	//The context object. This object owns the handles to all connected realsense devices.
	rs::context ctx;
	rs::device * dev;
	uint16_t * depth_frame;

	uint16_t one_meter;

	/**
	* process the pointcloud and flag obstacles
	*/
	void process(const uint16_t*  cloud);

	/**
	* returns true if the function counted 'mMinBlobSize' points in the given range. if freespace is true, the X range is being ignored.
	*/
	bool enoughPointsDetected(const uint16_t*  cloud, float targetZ, bool freespace);

	/**
	* Calc the direction to turn
	*/
	int calcDirection(const uint16_t*  cloud, float targetZ);



	friend fido_camera &the_camera();
};

fido_camera &the_camera();



