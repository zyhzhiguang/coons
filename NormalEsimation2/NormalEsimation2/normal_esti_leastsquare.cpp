#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/features/normal_esti_leastsquare.h>
#include <pcl/features/impl/normal_esti_leastsquare.hpp>

// Instantiations of specific point types
#ifdef PCL_ONLY_CORE_POINT_TYPES
PCL_INSTANTIATE_PRODUCT(NormalEstimation2, ((pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointXYZRGB)(pcl::PointXYZRGBA)(pcl::PointNormal))((pcl::Normal)(pcl::PointNormal)(pcl::PointXYZRGBNormal)))
#else
PCL_INSTANTIATE_PRODUCT(NormalEstimation2, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES))
#endif

