//
//  defs.h
//
//  Created by Fons Kuijk on 12-12-18.
//

#ifndef cwipc_realsense2_defs_h
#define cwipc_realsense2_defs_h

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>

//
// Definitions of types used across cwipc_realsense2, cwipc_codec and cwipc_util.
//
#if 0
typedef pcl::PointXYZRGB cwipc_pcl_point;
typedef  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cwipc_pcl_pointcloud;
inline cwipc_pcl_pointcloud new_cwipc_pcl_pointcloud(void) { return cwipc_pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>); }
#endif
#include "cwipc_util/api_pcl.h"

#endif /* cwipc_realsense2_defs_h */
