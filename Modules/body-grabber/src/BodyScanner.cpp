#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

#include <pcl_addons/io/openni_human_grabber.h>

#include <fstream>
#include <string>
using namespace std;

#include <Body/Builder.h>



class BodyScannerApp {
public:
	BodyScannerApp(int argc, char** argv)
		//: grab_viewer("Body Scanner > Input")
		: viewer_("Body Scanner")
		, showed_first_cloud_(false)
		//, output_viewer("Body Scanner > Output")
		, bodyBuilder(&viewer_, &viewer_mutex_) {
		record = argc > 1;
		if (record)
			basename = argv[1];
		viewer_.addCoordinateSystem(1.0);
		viewer_.initCameraParameters();

		viewer_.registerKeyboardCallback(keyboardCallback, this);
	}

	void body_cloud_cb_(
			//const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
			const Body::BodyPointCloud::ConstPtr cloud,
			const boost::shared_ptr<BodyScanner::OpenNIHumanGrabber::BodyPose> body_pose) {

		if(body_pose->skeleton_pose != NULL)
			bodyBuilder.pushSample(cloud, body_pose->skeleton_pose);

		viewer_mutex_.lock();
			if (!viewer_.wasStopped()) {
				//viewer_.removeShape("live_cloud");
				if(!showed_first_cloud_)
				{
					//viewer_.addPointCloud(cloud, "live_cloud");
					showed_first_cloud_ = true;
				}
				else
				{
					//viewer_.updatePointCloud(cloud, "live_cloud");
				}
			}
		viewer_mutex_.unlock();

		/*if (record && body_pose->skeleton_pose != NULL) {
			static int frame = 0;

			char pcd_filename[strlen(basename) + 11];
			sprintf(pcd_filename, "%s_%05d.pcd", basename, frame);
			writer.writeBinary(string(pcd_filename), *cloud);

			char skeleton_filename[strlen(basename) + 21];
			sprintf(skeleton_filename, "%s_skeleton_%05d.yaml", basename, frame);
			skeleton_writer.open(skeleton_filename);
			Body::Skeleton::Pose pose = *body_pose->skeleton_pose;
			skeleton_writer << pose.toYaml() << endl;
			skeleton_writer.close();

			printf("saved %s and %s\n", pcd_filename, skeleton_filename);
			frame++;
		}*/

		//if (record && body)

	}

	static void keyboardCallback(const pcl::visualization::KeyboardEvent &keyboardEvent, void *data)
	{
		BodyScannerApp* scanner = (BodyScannerApp*) data;
		if(keyboardEvent.getKeyCode() == ' ' && keyboardEvent.keyUp())
		{
			std::stringstream obj_filename; obj_filename << scanner->basename << ".obj";
			scanner->bodyBuilder.saveObj(obj_filename.str());
		}
	}

	void run() {
		BodyScanner::OpenNIHumanGrabber* interface = new BodyScanner::OpenNIHumanGrabber();

		boost::function<void(
				//const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&,
				const Body::BodyPointCloud::ConstPtr,
				const boost::shared_ptr<BodyScanner::OpenNIHumanGrabber::BodyPose>)> g =
				boost::bind(&BodyScannerApp::body_cloud_cb_, this, _1, _2);

		interface->registerCallback(g);

		interface->start();

		bool viewer_was_stopped = false;
		while (!viewer_was_stopped /*&& !output_viewer.wasStopped()*/) {
			boost::this_thread::sleep(boost::posix_time::microseconds(10000));
			viewer_mutex_.lock();

				viewer_.spinOnce(100);

				viewer_was_stopped = viewer_.wasStopped();
			viewer_mutex_.unlock();
		}

		interface->stop();
	}

	//pcl::visualization::CloudViewer grab_viewer;
	pcl::visualization::PCLVisualizer viewer_;
	boost::mutex viewer_mutex_;
	Body::Builder bodyBuilder;
	pcl::PCDWriter writer;
	ofstream skeleton_writer;
	bool record;
	char* basename;

	bool showed_first_cloud_;
};

int main(int argc, char** argv) {
	BodyScannerApp v(argc, argv);
	v.run();
	return 0;
}
