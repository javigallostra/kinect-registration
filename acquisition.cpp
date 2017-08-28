#include "acquisition.h"


int ImageGrabber::initializeGrabber ()
{
	// Check for connected devices
	if (freenect2.enumerateDevices() == 0)
	{
		std::cerr << "Kinect v2 not found." << std::endl;
		return 0;
	}
	if (verbose) {std::cout << "Kinect v2 found..." << std::endl;}
	// Get device serial
	std::string serial = freenect2.getDefaultDeviceSerialNumber();
	if (verbose) {std::cout << " Kinect v2 Serial: " << serial << std::endl;}
	// Open device
	dev = freenect2.openDevice(serial);
	if (dev == 0)
	{
		std::cerr << "Failure in opening Kinect v2 device." << std::endl;
		return 0;
	}
	if (verbose) {std::cout << "Grabber initialized..." << std::endl;}
	return 1;
}

void ImageGrabber::startDevice ()
{
	// Set device listener
	listener =  new libfreenect2::SyncMultiFrameListener (libfreenect2::Frame::Color |
                                          libfreenect2::Frame::Depth |
                                          libfreenect2::Frame::Ir);
	dev->setColorFrameListener(listener);
	dev->setIrAndDepthFrameListener(listener);
	// Start device
	dev->start();
	if (verbose) {std::cout << "Kinectv2 device started..." << std::endl;}
}

void ImageGrabber::initParams ()
{
	// Numeric parameters
	image_counter = 0;
	image_width = 400;
	image_height = 400;
	width_gap = (512 - image_width) / 2;
	height_gap = (424 - image_height) / 2;
	// Variables used for registration
	registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	undistorted_frame = new libfreenect2::Frame (512, 424, 4);
	registered_frame = new libfreenect2::Frame (512, 424, 4);
	frame_cloud.reset(new PointCloud);
	// Set filter verbose
	filters.setVerbose(verbose);
}

const char * ImageGrabber::generateFilename ()
{
	// Convert int to string
	std::ostringstream number; 
	number << image_counter;
	std::string image_number = number.str();
	// Generate image number as 0000, 0001...
	for (int i = 10; i < 1001; i *= 10)
	{
		if (image_counter < i)
		{
			image_number = "0" + image_number;
		}
	}
	// Add .pcd extension
	std::string filename = "image" + image_number + ".pcd";
	return filename.c_str();
}


PointCloud::Ptr ImageGrabber::getCurrentFrameCloud ()
{
	PointCloud::Ptr cloud_out (new PointCloud);
	pcl::copyPointCloud(*frame_cloud, *cloud_out);
	return cloud_out;
}

void ImageGrabber::captureFrame ()
{
	PointCloud::Ptr raw_cloud (new PointCloud);
	// Release previous frames
	listener->release(frames);
	// Wait for frames
	listener->waitForNewFrame(frames);
	// Get frames
	rgb_frame = frames[libfreenect2::Frame::Color];
	ir_frame = frames[libfreenect2::Frame::Ir];
	depth_frame = frames[libfreenect2::Frame::Depth];
	// Register depth to rgb
	registration->apply(rgb_frame, depth_frame, undistorted_frame, registered_frame);
	// Fill pcl cloud
	raw_cloud->width = image_width;
	raw_cloud->height = image_height;
	raw_cloud->is_dense = false;
	raw_cloud->points.resize (raw_cloud->width * raw_cloud->height);
	float X, Y, Z, RGB;
	int counter = 0;
	for (int xi = width_gap; xi < (registered_frame->width - width_gap); xi++)
	{
		for (int yi = height_gap; yi < (registered_frame->height - height_gap); yi++)
		{
			registration->getPointXYZRGB(undistorted_frame, registered_frame, yi, xi, X, Y, Z, RGB);
			raw_cloud->points[counter].x = X;
			raw_cloud->points[counter].y = Y;
			raw_cloud->points[counter].z = Z;
			raw_cloud->points[counter].rgb = RGB;
			counter++;
		}
	}
	// Filter raw cloud into frame_cloud
	filters.applyAllFilters(raw_cloud, frame_cloud);
}

void ImageGrabber::saveFrameAsPCD ()
{
	// Generate filename
	filename = generateFilename();
	if (verbose) {std::cout << filename << " saved." << std::endl;}
	// Save file
	pcd_writer.writeBinary(filename, *frame_cloud);
	// Increment countetr
	image_counter += 1;
}

void ImageGrabber::stopDevice ()
{
	// Stop Kinectv2 device
	dev->stop();
	dev->close();
}

ImageGrabber::ImageGrabber (bool verb)
{
	// Class constructor
	verbose = verb;
	if (initializeGrabber() == 0 )
	{
		std::cerr << "Failure in initializing Grabber." << std::endl;
	} else {
		if (not verbose) {libfreenect2::setGlobalLogger(NULL);}
		startDevice();
		initParams();
		if (verbose) {std::cout << "Grabber initialized correctly." << std::endl;}
	}
}

ImageGrabber::~ImageGrabber ()
{
	// Class destructor
	stopDevice();
	if (verbose)
	{
		std::cout << "Device stopped." << std::endl;
		std::cout << "Grabber destructed." << std::endl;
	}
}