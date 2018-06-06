#include "GraphMatching.h"

GraphMatching::GraphMatching(pcl::PointCloud<pcl::PointXYZRGB>& keypoints1, pcl::PointCloud<pcl::PointXYZRGB>& keypoints2, const pcl::Correspondences& correspondence)
	:_m(keypoints1.size()*keypoints2.size(), keypoints1.size()*keypoints2.size()),
	_keypoints1(keypoints1), _keypoints2(keypoints2), _nrow(keypoints1.size()*keypoints2.size()), _ncol(keypoints1.size()*keypoints2.size())
{
	double threshold = 0.01;

	for (int i = 0; i < correspondence.size(); ++i)
	{
		for (int j = i + 1; j < correspondence.size(); ++j)
		{
			int model1_kp1 = correspondence[i].index_query;
			int model1_kp2 = correspondence[j].index_query;
			int model2_kp1 = correspondence[i].index_match;
			int model2_kp2 = correspondence[j].index_match;

			if (model1_kp1 == model1_kp2 || model2_kp1 == model2_kp2)
				continue;

			int index_ij1 = model1_kp1*keypoints2.size() + model2_kp1;
			int index_ij2 = model1_kp2*keypoints2.size() + model2_kp2;
			assert(index_ij1 != index_ij2);

			float dis_model1_x = keypoints1.points[model1_kp1].x - keypoints1.points[model1_kp2].x;
			float dis_model1_y = keypoints1.points[model1_kp1].y - keypoints1.points[model1_kp2].y;
			float dis_model1_z = keypoints1.points[model1_kp1].z - keypoints1.points[model1_kp2].z;
			if (fabs(dis_model1_x) < 0.01 && fabs(dis_model1_y) < 0.01 && fabs(dis_model1_z) < 0.01)
				continue;

			float dis_model2_x = keypoints2.points[model2_kp1].x - keypoints2.points[model2_kp2].x;
			float dis_model2_y = keypoints2.points[model2_kp1].y - keypoints2.points[model2_kp2].y;
			float dis_model2_z = keypoints2.points[model2_kp1].z - keypoints2.points[model2_kp2].z;
			if (fabs(dis_model2_x) < 0.01 && fabs(dis_model2_y) < 0.01 && fabs(dis_model2_z) < 0.01)
				continue;

			float dis_model1 = sqrt(dis_model1_x*dis_model1_x + dis_model1_y*dis_model1_y + dis_model1_z*dis_model1_z);
			float dis_model2 = sqrt(dis_model2_x*dis_model2_x + dis_model2_y*dis_model2_y + dis_model2_z*dis_model2_z);

			if (fabs(dis_model1 - dis_model2) >= 3 * threshold)
				continue;

			int row = index_ij1;
			int col = index_ij2;
			float val = 4.5 - (dis_model1 - dis_model2)*(dis_model1 - dis_model2) / (2 * threshold*threshold);

			_m(row, col) = val;
			_m(col, row) = val;

		}
	}
}



pcl::CorrespondencesPtr GraphMatching::ComputeCorrespondenceByEigenVec(int best_num /* = 4 */)
{

	pcl::CorrespondencesPtr cor4(new pcl::Correspondences);
	std::vector<int> result4;

	arma::fvec eigval;
	arma::fmat eigvec;
	arma::eigs_sym(eigval, eigvec, _m, 1, "la");

	if (eigvec.size() <= 0)
		return cor4;

	result4.resize(eigvec.size());
	for (int i = 0; i < result4.size(); ++i)
		result4[i] = 0;

	for (int i = 0; i < eigvec.size(); i++)
		if (eigvec(i) < 0)
			eigvec(i) = -eigvec(i);


	int num = 0;
	arma::fmat::iterator biggest = std::max_element(eigvec.begin(), eigvec.end());
	while (*biggest > 0.001)
	{
		// only use the best num correspondences
		if (num >= best_num)
			break;

		int index = std::distance(eigvec.begin(), biggest);
		result4[index] = 1;
		int i = index / _keypoints2.size();
		int j = index % _keypoints2.size();

		int start = i*_keypoints2.size();
		for (int inx = 0; inx < _keypoints2.size(); inx += 1)
			eigvec(start + inx) = 0.0;

		for (int inx = j; inx < _keypoints1.size()*_keypoints2.size(); inx += _keypoints2.size())
			eigvec(inx) = 0.0;

		biggest = std::max_element(eigvec.begin(), eigvec.end());
		num++;
	}

	// convert to pair
	if (num < best_num)
	{
		return cor4;
	}
	for (int i = 0; i < result4.size(); ++i)
	{
		if (result4[i] == 1)
		{
			pcl::Correspondence c;
			c.index_query = i / _keypoints2.size();
			c.index_match = i % _keypoints2.size();
			cor4->push_back(c);
		}
	}

	bool valid = true;
	for (int i = 0; i < cor4->size(); i++)
	{
		int frame1_query1 = cor4->at(i).index_query;
		int frame2_match1 = cor4->at(i).index_match;

		for (int j = i + 1; j < cor4->size(); ++j)
		{
			int frame1_query2 = cor4->at(j).index_query;
			int frame2_match2 = cor4->at(j).index_match;

			if (frame1_query1 == frame1_query2 || frame2_match1 == frame2_match2)
			{
				valid = false;
				break;
			}

			float dis_model1_x = _keypoints1.points[frame1_query1].x - _keypoints1.points[frame1_query2].x;
			float dis_model1_y = _keypoints1.points[frame1_query1].y - _keypoints1.points[frame1_query2].y;
			float dis_model1_z = _keypoints1.points[frame1_query1].z - _keypoints1.points[frame1_query2].z;
			if (fabs(dis_model1_x) < 0.01 && fabs(dis_model1_y) < 0.01 && fabs(dis_model1_z) < 0.01)
			{
				valid = false;
				break;
			}

			float dis_model2_x = _keypoints2.points[frame2_match1].x - _keypoints2.points[frame2_match2].x;
			float dis_model2_y = _keypoints2.points[frame2_match1].y - _keypoints2.points[frame2_match2].y;
			float dis_model2_z = _keypoints2.points[frame2_match1].z - _keypoints2.points[frame2_match2].z;
			if (fabs(dis_model2_x) < 0.01 && fabs(dis_model2_y) < 0.01 && fabs(dis_model2_z) < 0.01)
			{
				valid = false;
				break;
			}
		}
	}
	if (!valid)
	{
		cor4->clear();
		return cor4;
	}


	// judge 4 points in the same plane
	std::vector<pcl::PointXYZ> arr1;
	std::vector<pcl::PointXYZ> arr2;
	pcl::PointXYZ p[4];
	for (int i = 0; i < 4; ++i)
	{
		int frame1_query = cor4->at(i).index_query;
		int frame2_match = cor4->at(i).index_match;

		pcl::PointXYZ p1, p2;
		p1.x = _keypoints1.points[frame1_query].x;	p1.y = _keypoints1.points[frame1_query].y;	p1.z = _keypoints1.points[frame1_query].z;
		p2.x = _keypoints2.points[frame2_match].x;	p2.y = _keypoints2.points[frame2_match].y;	p2.z = _keypoints2.points[frame2_match].z;
		arr1.push_back(p1);
		arr2.push_back(p2);
	}
	int add_num = 0;
	while ((judge_pointn_plane(arr1) || judge_pointn_plane(arr2)) && add_num < 16 && *biggest > 0.001)
	{
		// add new edge
		int index = std::distance(eigvec.begin(), biggest);
		result4[index] = 1;
		int i = index / _keypoints2.size();
		int j = index % _keypoints2.size();

		int start = i*_keypoints2.size();
		for (int inx = 0; inx < _keypoints2.size(); inx += 1)
			eigvec(start + inx) = 0.0;

		for (int inx = j; inx < _keypoints1.size()*_keypoints2.size(); inx += _keypoints2.size())
			eigvec(inx) = 0.0;

		// convert to pair
		pcl::Correspondence c;
		c.index_query = index / _keypoints2.size();
		c.index_match = index % _keypoints2.size();
		cor4->push_back(c);

		int frame1_query = index / _keypoints2.size();
		int frame2_match = index % _keypoints2.size();

		pcl::PointXYZ p1, p2;
		p1.x = _keypoints1.points[frame1_query].x;	p1.y = _keypoints1.points[frame1_query].y;	p1.z = _keypoints1.points[frame1_query].z;
		p2.x = _keypoints2.points[frame2_match].x;	p2.y = _keypoints2.points[frame2_match].y;	p2.z = _keypoints2.points[frame2_match].z;
		arr1.push_back(p1);
		arr2.push_back(p2);

		biggest = std::max_element(eigvec.begin(), eigvec.end());
		add_num++;
	}


	// transformation keypoints1 to keypoints 2
	Eigen::Matrix4f	transformation2 = ComputeRigid(cor4, _keypoints1, _keypoints2);

	_transformation = transformation2;
	return cor4;
}



Eigen::Matrix4f GraphMatching::ComputeRigid(pcl::CorrespondencesConstPtr correspondence, const pcl::PointCloud<pcl::PointXYZRGB>& keypoints1, const pcl::PointCloud<pcl::PointXYZRGB>& keypoints2)
{
	int point_num = correspondence->size();

	//frame 1
	std::vector<int> frame1_p_index;
	frame1_p_index.resize(point_num);
	for (int i = 0; i < point_num; ++i)
		frame1_p_index[i] = correspondence->at(i).index_query;

	std::vector<pcl::PointXYZRGB> frame1_pp;
	frame1_pp.resize(point_num);
	for (int i = 0; i < point_num; ++i)
		frame1_pp[i] = keypoints1.at(frame1_p_index[i]);

	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> frame1_p;
	frame1_p.resize(point_num);
	for (int i = 0; i < point_num; ++i)
		frame1_p[i] << frame1_pp[i].x, frame1_pp[i].y, frame1_pp[i].z;

	Eigen::Vector3f frame1_p_center(0.0, 0.0, 0.0);
	for (int i = 0; i < point_num; ++i)
	{
		frame1_p_center(0) += frame1_p[i](0);
		frame1_p_center(1) += frame1_p[i](1);
		frame1_p_center(2) += frame1_p[i](2);
	}
	frame1_p_center = frame1_p_center / point_num;


	//frame 2
	std::vector<int> frame2_p_index;
	frame2_p_index.resize(point_num);
	for (int i = 0; i < point_num; ++i)
		frame2_p_index[i] = correspondence->at(i).index_match;

	std::vector<pcl::PointXYZRGB> frame2_pp;
	frame2_pp.resize(point_num);
	for (int i = 0; i < point_num; ++i)
		frame2_pp[i] = keypoints2.at(frame2_p_index[i]);

	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> frame2_p;
	frame2_p.resize(point_num);
	for (int i = 0; i < point_num; ++i)
		frame2_p[i] << frame2_pp[i].x, frame2_pp[i].y, frame2_pp[i].z;

	Eigen::Vector3f frame2_p_center(0.0, 0.0, 0.0);
	for (int i = 0; i < point_num; ++i)
	{
		frame2_p_center(0) += frame2_p[i](0);
		frame2_p_center(1) += frame2_p[i](1);
		frame2_p_center(2) += frame2_p[i](2);
	}
	frame2_p_center = frame2_p_center / point_num;


	//svd
	Eigen::MatrixXf frame1_c_mat(3, point_num);
	for (int col = 0; col < point_num; ++col)
		for (int row = 0; row < 3; ++row)
			frame1_c_mat(row, col) = frame1_p_center(row);

	Eigen::MatrixXf mat1(3, point_num);
	for (int col = 0; col < point_num; ++col)
		for (int row = 0; row < 3; ++row)
			mat1(row, col) = frame1_p[col](row);

	mat1 = mat1 - frame1_c_mat;


	Eigen::MatrixXf w(point_num, point_num);
	for (int row = 0; row < point_num; ++row)
		for (int col = 0; col < point_num; ++col)
			if (row == col)
				w(row, col) = 1.0;
			else
				w(row, col) = 0.0;



	Eigen::MatrixXf frame2_c_mat(point_num, 3);
	for (int row = 0; row < point_num; ++row)
		for (int col = 0; col < 3; ++col)
			frame2_c_mat(row, col) = frame2_p_center(col);

	Eigen::MatrixXf mat2(point_num, 3);
	for (int row = 0; row < point_num; ++row)
		for (int col = 0; col < 3; ++col)
			mat2(row, col) = frame2_p[row](col);

	mat2 = mat2 - frame2_c_mat;


	Eigen::MatrixXf mat(3, 3);
	mat = mat1*w*mat2;
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

	auto U = svd.matrixU();
	auto V = svd.matrixV();
	auto singular_value = svd.singularValues();

	Eigen::Matrix3f r = V*U.transpose();
	Eigen::Vector3f t = frame2_p_center - r*frame1_p_center;


	Eigen::Matrix4f result;
	result <<
		r(0, 0), r(0, 1), r(0, 2), t(0),
		r(1, 0), r(1, 1), r(1, 2), t(1),
		r(2, 0), r(2, 1), r(2, 2), t(2),
		0, 0, 0, 1;

	return result;
}