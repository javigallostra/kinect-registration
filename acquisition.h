#include "typedefs.h"

#include <stdio.h> // basics
#include <iostream>

#include <fstream>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp> // for image visualization
#include <pcl/visualization/pcl_visualizer.h> // for pcl visualization

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <signal.h> // for exiting the loop

#include <cmath>


class ImageGrabber
{
	private:
		int image_counter;
		int image_width;
		int image_height;
		int width_gap;
		int height_gap;
		
		const char *filename;

		bool verbose;
		
		libfreenect2::Freenect2 freenect2;
		libfreenect2::Freenect2Device *dev;
		libfreenect2::FrameMap frames;
		libfreenect2::SyncMultiFrameListener *listener;
		libfreenect2::Registration *registration;
		libfreenect2::Frame *rgb_frame, *ir_frame, *depth_frame;
		libfreenect2::Frame *undistorted_frame, *registered_frame;

		PointCloud::Ptr frame_cloud;
		pcl::PCDWriter pcd_writer;

		int initializeGrabber ();

		void startDevice ();

		void initParams ();

		const char *generateFilename ();

	public:

		PointCloud::Ptr getCurrentFrameCloud ();

		void captureFrame ();

		void saveFrameAsPCD ();

		void stopDevice ();

		ImageGrabber (bool verb);

		virtual ~ImageGrabber ();
};