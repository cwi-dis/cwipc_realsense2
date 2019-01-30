//
//  multiFrame.cpp
//
//  Created by Fons Kuijk on 23-04-18
//

#include "multiFrame.hpp"
#include "utils.h"

/**/
const int color_width = 1280;
const int color_height = 720;
const int color_fps = 30;
const int depth_width = 1280;
const int depth_height = 720;
const int depth_fps = 30;
/*
const int color_width = 640;
const int color_height = 360;
const int color_fps = 60;
const int depth_width = 640;
const int depth_height = 360;
const int depth_fps = 60;
/**/


// Configure and initialize caputuring of one camera
void multiFrame::camera_start(cameradata camera_data)
{
	cout << "starting camera ser no: " << camera_data.serial << '\n';

	rs2::config cfg;
	cfg.enable_device(camera_data.serial);
	cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_RGB8, color_fps);
	cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, depth_fps);

	camera_data.pipe.start(cfg);		// Start streaming with the configuration just set
}

// get new frames from the camera and update the pointcloud of the camera's data 
void multiFrame::camera_action(cameradata camera_data)
{
	if (!do_capture)
		return;

	rs2::pointcloud pc;
	rs2::points points;

#ifdef POLLING
	// Poll to find if there is a next set of frames from the camera
	rs2::frameset frames;
	if (!camera_data.pipe.poll_for_frames(&frames))
		return;
#else
	// Wait to find if there is a next set of frames from the camera
	auto frames = camera_data.pipe.wait_for_frames();
#endif

	auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();
	float minz = 100.0f, maxz, minx;

	camera_data.cloud->clear();

	// Tell points frame to map to this color frame
	pc.map_to(color); // NB: This does not align the frames. That should be handled by setting resolution of cameras
	points = pc.calculate(depth);

	// Generate new vertices and color vector
	auto vertices = points.get_vertices();

	unsigned char *colors = (unsigned char*)color.get_data();

	// Set the background removal window
	for (int i = 0; i < points.size(); i++) {
		if (vertices[i].z != 0 && minz > vertices[i].z) {
			minz = vertices[i].z;
			minx = vertices[i].x;
		}
	}
	maxz = 0.8f + minz;

	// Make PointCloud
	for (int i = 0; i < points.size(); i++) {
		float x = minx - vertices[i].x; x *= x;
		float z = vertices[i].z;
		if (minz < z && z < maxz - x) { // Simple background removal, horizontally parabolic, vertically straight.
			PointT pt;
			pt.x = vertices[i].x;
			pt.y = -vertices[i].y;
			pt.z = -z;
			int pi = i * 3;
			pt.r = colors[pi];
			pt.g = colors[pi + 1];
			pt.b = colors[pi + 2];
			if (!green_screen || noChromaRemoval(&pt)) // chromakey removal
				camera_data.cloud->push_back(pt);
		}
	}
}

void multiFrame::merge_views(boost::shared_ptr<PointCloudT> cloud_ptr)
{
	PointCloudT *aligned_cld = new PointCloudT();

	cloud_ptr->clear();
	for (cameradata ccfg : CameraData) {
		PointCloudT *cam_cld = ccfg.cloud.get();
		if (cam_cld->size() > 0) {
			transformPointCloud(*cam_cld, *aligned_cld, *ccfg.trafo);
			*cloud_ptr.get() += *aligned_cld;
		}
	}
 	if (spatial_resolution > 0) {
#ifdef DEBUG
		cout << "Points before reduction: " << cloud_ptr.get()->size() << endl;
#endif
		VoxelGrid<PointT> grd;
		grd.setInputCloud(cloud_ptr);
		grd.setLeafSize(spatial_resolution, spatial_resolution, spatial_resolution);
		grd.setSaveLeafLayout(true);
		grd.filter(*cloud_ptr);
#ifdef DEBUG
		cout << "Points after reduction: " << cloud_ptr.get()->size() << endl;
#endif
	}
	if (tiling && tiling_resolution > 0) {
		make_tiles(cloud_ptr);
	}
}

// Make tiles of the merged pointcloud
void multiFrame::make_tiles(boost::shared_ptr<PointCloudT> cloud_ptr)
{
	boost::shared_ptr<PointCloudT> voxels(new PointCloudT());
	// Create a (lower resolution) voxelcloud based on the merged cloud
	pcl::VoxelGrid<PointT> grid;
	grid.setInputCloud(cloud_ptr);
	grid.setLeafSize(tiling_resolution, tiling_resolution, tiling_resolution);
	grid.setSaveLeafLayout(true);
	grid.filter(*voxels);
#ifdef DEBUG
	cout << "Number of voxels " << voxels.get()->size() << endl;
#endif
	// Find which camera's contributed to each voxel
	int maxlbl = 0;
	for (int i = 0, lbl = 1; i < CameraData.size(); i++, lbl = lbl << 1) {
		// lbl is the one bit that is assigned to a camera
		for (auto pnt : CameraData[i].cloud->points) {
			// getCentroidIndex returns the index of the voxel point in which pnt contributed
			/*PointTL *p = &voxels->points[grid.getCentroidIndex(pnt)];
			p->label |= lbl;
			if (p->label > maxlbl)
				maxlbl = p->label;/**/
		}
	} 
	/*
	// Now assign the labels to the points in the merged cloud
	for (int i = 0; i < cloud_ptr->size(); i++)
		cloud_ptr->points[i].label = voxels->points[grid.getCentroidIndex(cloud_ptr->points[i])].label;

	// Create individual tiles based on the point labels
	for (int i = 0; i < maxlbl; i++) {
		boost::shared_ptr<PointCloudT> tile(new PointCloudT());
		CloudTiles.push_back(tile);
	}

	// Assign points to the tiles
	for (auto pl : cloud_ptr->points) {
		PointT *p = new PointT();
		p->x = pl.x;
		p->y = pl.y;
		p->z = pl.z;
		p->r = pl.r;
		p->g = pl.g;
		p->b = pl.b;
		//CloudTiles[pl.label - 1]->push_back(*p);
 	}/**/
}


// API function that triggers the capture and returns the merged pointcloud and timestamp
void multiFrame::get_pointcloud(uint64_t *timestamp, void **pointcloud)
{
	*timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	if (CameraData.size() > 0) {
		for (cameradata ccfg : CameraData)
			camera_action(ccfg);
		merge_views(RingBuffer[ring_index]);
		if (RingBuffer[ring_index].get()->size() > 0) {
#ifdef DEBUG
			cout << "capturer produced a merged cloud of " << RingBuffer[ring_index].get()->size() << " points in ringbuffer " << ring_index << "\n";
#endif
			*pointcloud = reinterpret_cast<void *> (&(RingBuffer[ring_index]));
		}
		else {
#ifdef DEBUG
			cout << "\nWARNING: capturer did get an empty pointcloud\n\n";
#endif
			// HACK to make sure the encoder does not get an empty pointcloud 
			PointT point;
			point.x = 1.0;
			point.y = 1.0;
			point.z = 1.0;
			point.rgb = 0.0;
			RingBuffer[ring_index]->points.push_back(point);
			*pointcloud = reinterpret_cast<void *> (&(RingBuffer[ring_index]));
		}
	}
	else {	// return a spinning generated mathematical pointcloud
		angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
		transformPointCloud(*GeneratedPC, *RingBuffer[ring_index], transform);
		*pointcloud = reinterpret_cast<void *> (&(RingBuffer[ring_index]));
	}
	ring_index = ring_index < ringbuffer_size - 1 ? ++ring_index : 0;
}


///////////////////////////
// class captureIt stuff //
///////////////////////////


void captureIt::getPointCloud(uint64_t *timestamp, void **pointcloud) {
	static multiFrame mFrame;

#ifdef DEBUG
	cout << "captureIt is asked for a pointcloud\n";
#endif

	mFrame.get_pointcloud(timestamp, pointcloud);

#ifdef DEBUG
	boost::shared_ptr<PointCloudT> captured_pc;
	captured_pc = *reinterpret_cast<boost::shared_ptr<PointCloudT>*>(*pointcloud);
	cout << "captureIt handed over a pointcloud of " << captured_pc.get()->size() << " points\n";
	cout.flush();
#endif
}

extern "C" void __declspec(dllexport) getPointCloud(uint64_t *timestamp, void **pointcloud) {
	captureIt captureit;
	captureit.getPointCloud(timestamp, pointcloud);
}
