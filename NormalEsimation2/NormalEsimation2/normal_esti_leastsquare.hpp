#ifndef PCL_FEATURES_IMPL_NORMAL_ESTI_LEASTSQUARE_H_
#define PCL_FEATURES_IMPL_NORMAL_ESTI_LEASTSQUARE_H_

#include <pcl/features/normal_esti_leastsquare.h>

template <typename PointInT, typename PointOutT> void
	pcl::NormalEstimation2<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
	// Allocate enough space to hold the results
	// \note This resize is irrelevant for a radiusSearch ().
	std::vector<int> nn_indices (k_);
	std::vector<float> nn_dists (k_);

	output.is_dense = true;
	// Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
	if (input_->is_dense)
	{
		// Iterating over the entire index vector
		for (size_t idx = 0; idx < indices_->size (); ++idx)
		{
			if (this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
			{
				output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();

				output.is_dense = false;
				continue;
			}

			computePointNormal (*surface_, nn_indices,
				output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature);

			//flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
			//	output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);

		}
	}
}

#define PCL_INSTANTIATE_NormalEstimation2(T,NT) template class PCL_EXPORTS pcl::NormalEstimation2<T,NT>;

#endif  //PCL_FEATURES_IMPL_NORMAL_ESTI_LEASTSQUARE_H_
