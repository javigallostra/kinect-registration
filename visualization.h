#include "typedefs.h"

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

class Viewer
{
	private:

		static void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* pressedID_void);
		
		void displayClouds (PointCloud::Ptr grabberCloud, PointCloud::Ptr sourceCloud, PointCloud::Ptr targetCloud, PointCloud::Ptr composCloud, PointCloud::Ptr sourceKp, PointCloud::Ptr targetKp);

		int vgrab, vtarg, vsour, vcorr, vfcorr, vcomp, pressedID;

		Visualizer::Ptr viewer;

		Eigen::Matrix4f correspondence_source_transform;

		Eigen::Matrix4f correspondence_target_transform;

	public:

		Viewer (PointCloud::Ptr grabberCloud, PointCloud::Ptr sourceCloud, PointCloud::Ptr targetCloud, PointCloud::Ptr composCloud, PointCloud::Ptr sourceKp, PointCloud::Ptr targetKp);

		Visualizer::Ptr getViewer ();

		void updateGrabber (PointCloud::Ptr grabberCloud);

		void updateClouds (PointCloud::Ptr sourceCloud, PointCloud::Ptr targetCloud, PointCloud::Ptr composCloud, PointCloud::Ptr sourceKp, PointCloud::Ptr targetKp);

		void updateCorrespondences (pcl::CorrespondencesPtr correspondences, PointCloud::Ptr sourceKp, PointCloud::Ptr targetKp);

		void updateCorrespondenceTransform (Eigen::Matrix4f new_transform);

		void addFinalMesh (pcl::PolygonMesh::Ptr mesh_in);

		void updateFPS (std::string fps);

		int getPressedID ();

		void setPressedID (int newID);
};