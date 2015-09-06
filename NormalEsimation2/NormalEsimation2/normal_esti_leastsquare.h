#ifndef PCL_NORMAL_ESTI_LEASTSQUARE_H_
#define PCL_NORMAL_ESTI_LEASTSQUARE_H_

#include <pcl/features/feature.h>

namespace pcl
{
	template <typename PointInT, typename PointOutT>
	class NormalEstimation2 : public Feature<PointInT, PointOutT>
	{
	public:
		using Feature<PointInT, PointOutT>::feature_name_;
		using Feature<PointInT, PointOutT>::input_;
		using Feature<PointInT, PointOutT>::indices_;
		using Feature<PointInT, PointOutT>::k_;
		using Feature<PointInT, PointOutT>::search_radius_;
		using Feature<PointInT, PointOutT>::search_parameter_;

		typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
		typedef typename Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;

		NormalEstimation2()
			: vpx_ (0)
			, vpy_ (0)
			, vpz_ (0)
			, u_matrix_ ()
			, use_sensor_origin_ (false)
		{
			feature_name_ = "NormalEstimation2";
		}

     /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      virtual inline void 
      setInputCloud (const PointCloudConstPtr &cloud)
      {
        input_ = cloud;
        if (use_sensor_origin_)
        {
          vpx_ = input_->sensor_origin_.coeff (0);
          vpy_ = input_->sensor_origin_.coeff (1);
          vpz_ = input_->sensor_origin_.coeff (2);
        }
      }

	   /** \brief 计算点的法向量，曲率暂时不求
	     * \param[in] 输入点云
	     * \param[in] 给定点的索引下标 
		 * \param[out] 法向量x分量 
		 * \param[out] 法向量y分量  
		 * \param[out] 法向量z分量  
		 * \param[out] 曲率 
	     */
	  inline void
		  computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, float &nx, float &ny, float &nz, float &curvature)
	  {
#define _METHOD1_
#ifdef _METHOD1_
		  u_matrix_.resize(k_ , 3);

		  Eigen::VectorXf z_vec(k_);
		  for (size_t i = 0, ind_num = indices.size(); i < ind_num; ++i)
		  {
			  size_t indice = indices[i];

			  u_matrix_(i, 0) = cloud.points[indice].x;
			  u_matrix_(i, 1) = cloud.points[indice].y;
			  u_matrix_(i, 2) = 1;

			  z_vec[i] = cloud.points[indice].z;
		  }
#if defined _DEBUG1
		  std::cout<<"u_matrix = "<<std::endl<<u_matrix_<<std::endl;
		  std::cout<<"z_vec = "<<std::endl<<z_vec<<std::endl;
#endif
		  EIGEN_ALIGN16 Eigen::Matrix3f utu_matrix = u_matrix_.transpose() * u_matrix_;
		  EIGEN_ALIGN16 Eigen::Vector3f solve_vec = utu_matrix.inverse() * u_matrix_.transpose() * z_vec;
#if defined _DEBUG1
		  std::cout<<"utu_matrix = "<<std::endl<<utu_matrix<<std::endl;
		  std::cout<<"solve_vec = "<<std::endl<<solve_vec<<std::endl;
#endif
		  EIGEN_ALIGN16 Eigen::Vector3f normal_vec(solve_vec[0], solve_vec[1], -1);
		  normal_vec /= normal_vec.squaredNorm();
		  nx = normal_vec[0];
		  ny = normal_vec[1];
		  nz = normal_vec[2];
#endif    //_METHOD1_

#ifdef _METHOD2_

		  Eigen::MatrixX4f mat_ext(k_, 4);
		  for (size_t i = 0, ind_num = indices.size(); i < ind_num; ++i)
		  {
			  size_t indice = indices[i];

			  mat_ext(i, 0) = cloud.points[indice].x;
			  mat_ext(i, 1) = cloud.points[indice].y;
			  mat_ext(i, 2) = cloud.points[indice].z;
			  mat_ext(i, 3) = 1;
		  }

		  Eigen::MatrixXf mat_mult_ext = mat_ext.transpose() * mat_ext;    //mat是3*k_矩阵，mat_ext是k_*4矩阵
		  // A*x = b, A.Row(0).cross(A.Row(1)) = eigenvector(0);
		  Eigen::Vector4f row_vec[3];
		  row_vec[0] = mat_mult_ext.row(0);
		  row_vec[1] = mat_mult_ext.row(1);
		  row_vec[2] = mat_mult_ext.row(2);
		  Eigen::Matrix3f i_cofactor, j_cofactor, k_cofactor, l_cofactor;    //计算叉积时，i、j、k、l分量的代数余子式

		  for (size_t row_index = 0; row_index < 3; ++row_index)
		  {
			  for (size_t col_index = 0; col_index < 3; ++col_index)
			  {
				  i_cofactor(row_index , col_index) = row_vec[row_index][col_index + 1];

				  if (col_index < 1)
				  {
					  j_cofactor(row_index , col_index) = row_vec[row_index][col_index];
				  } 
				  else
				  {
					  j_cofactor(row_index , col_index) = row_vec[row_index][col_index + 1];
				  }

				  if (col_index < 2)
				  {
					  k_cofactor(row_index , col_index) = row_vec[row_index][col_index];
				  } 
				  else
				  {
					  k_cofactor(row_index , col_index) = row_vec[row_index][col_index + 1];
				  }

				  l_cofactor(row_index , col_index) = row_vec[row_index][col_index];
			  }
		  }

		  float i_dim = 0, j_dim = 0, k_dim = 0, l_dim = 0;
		  i_dim = i_cofactor.determinant();
		  j_dim = j_cofactor.determinant();
		  k_dim = k_cofactor.determinant();
		  l_dim = l_cofactor.determinant();
		  //Eigen::Vector4f b(0, 0, 0, 0);
		  //Eigen::Vector4f argument = mat_mult_ext.colPivHouseholderQr().solve(b);
		  //Eigen::Vector3f coeff_vec(argument[0], argument[1], argument[2]);
		  Eigen::Vector3f coeff_vec(i_dim, -j_dim, k_dim);
		  float len = coeff_vec.squaredNorm();
		  coeff_vec /= len;

		  nx = coeff_vec[0];
		  ny = coeff_vec[1];
		  nz = coeff_vec[2];
#endif    //_METHOD1_

	  }

	protected:
		        /** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
           * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
		float vpx_, vpy_, vpz_;

		/** whether the sensor origin of the input cloud or a user given viewpoint should be used.*/
		bool use_sensor_origin_;

		/** \brief 最小二乘法过程中的系数矩阵U, 是一个k_行， 3列的矩阵 */
		EIGEN_ALIGN16 Eigen::MatrixX3f u_matrix_;

	   /** \brief 计算点云的法向量，曲率暂时不求
	     * \param[out] 
	     */
	  void
	  computeFeature (PointCloudOut &output);
	  
	private:
      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
        * \param[out] output the output point cloud
        */
      void
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &) {}
	};
}

#endif    //PCL_NORMAL_ESTI_LEASTSQUARE_H_
