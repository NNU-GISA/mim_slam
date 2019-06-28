#include "mim_slam/util.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#define LOG_FLAG
#define KITTI_PATH

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> slamSyncPolicy;

class LaserMapping{
private:
	bool is_initial;
	int update_count;
	ros::Time stamp;

	#ifdef KITTI_PATH
		vector<vector<double>> kitti_path;
	#endif

	//点云定位
	Pose6D current_odom;
	Pose6D last_odom;
	Pose6D incremental_odom;
	Pose6D current_localization;
	Pose6D current_temp_localization;
	Pose6D last_localization;
	Pose6D incremental_localization;

	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn;
	pcl::PointCloud<pcl::PointXYZ>::Ptr query;
	pcl::PointCloud<pcl::PointXYZ>::Ptr reference;
	pcl::PointCloud<pcl::PointXYZ>::Ptr map_data;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aftMappingCloud;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map_octree;

	//闭环、图优化
	int key;
	gtsam::Pose3 odometry; //累积位姿，提取关键帧
	std::unique_ptr<gtsam::ISAM2> isam; //ISAM求解器
	gtsam::Values values; //图顶点
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> keyed_scans; //关键帧
	int last_closure_key;

	// 订阅，加入同步机制（激光/里程计）
	message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_ ;          // topic1 输入  
	message_filters::Subscriber<sensor_msgs::PointCloud2>* laser_sub_;     // topic2 输入  
	message_filters::Synchronizer<slamSyncPolicy>* sync_; 

	ros::Publisher pubLaserLocalizer;
	ros::Publisher pubLaserLocalizerCloud;
	ros::Publisher pubMapCloud;
	ros::Publisher pubGraphNode;
	tf2_ros::TransformBroadcaster tfbr;

public:
	LaserMapping(ros::NodeHandle& n): is_initial(false), update_count(0), key(0){
		laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZ>());
		query.reset(new pcl::PointCloud<pcl::PointXYZ>());
		reference.reset(new pcl::PointCloud<pcl::PointXYZ>());
		map_data.reset(new pcl::PointCloud<pcl::PointXYZ>());
		aftMappingCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

		map_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(map_octree_resolution));
		map_octree->setInputCloud(map_data);

		ros::NodeHandle nh(n);

		odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/laser_odom", 1);    //里程计
		laser_sub_  = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/laser_odom_cloud", 1);          //三维激光
		sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(10), *odom_sub_, *laser_sub_);  
		sync_->registerCallback(boost::bind(&LaserMapping::combineCallback,this, _1, _2)); 

		pubLaserLocalizer = nh.advertise<nav_msgs::Odometry> ("/laser_localizer", 1, false);
		pubLaserLocalizerCloud = nh.advertise<sensor_msgs::PointCloud2> ("/laser_localizer_cloud", 1,false);
		pubMapCloud = nh.advertise<sensor_msgs::PointCloud2> ("/map_data_cloud", 1,false);
		pubGraphNode = nh.advertise<visualization_msgs::Marker>("/graph_nodes", 1, false);
	
		//ISAM初始化
		gtsam::ISAM2Params parameters;
		parameters.relinearizeSkip = 1;
		parameters.relinearizeThreshold = 0.01;
		isam.reset(new gtsam::ISAM2(parameters));

		last_closure_key = std::numeric_limits<int>::min();

	}
	~LaserMapping(){
		#ifdef KITTI_PATH
			std::string path = "/home/zhaoming/KITTI/predict_pose.txt";
			SerialPath2File(path);
		#endif
		std::string map_name("saveMap.pcd");  
		pcl::PCDWriter writer;  
		writer.write(map_name, *map_data); 
	}


	void combineCallback(const nav_msgs::Odometry::ConstPtr odom, const sensor_msgs::PointCloud2::ConstPtr laser){

		if(! is_initial){
			stamp = laser->header.stamp;
			current_odom = Pose6D(Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z),
				Quaternion(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z));
			current_localization = current_odom;

			#ifdef KITTI_PATH
				vector<double> temp(9,0.0);
				current_localization.rot().toRotMatrix(temp);
				double q00 = temp[0]; double q01 = temp[1]; double q02 = temp[2];
				double q10 = temp[3]; double q11 = temp[4]; double q12 = temp[5];
				double q20 = temp[6]; double q21 = temp[7]; double q22 = temp[8];

				double q03 = current_localization.trans().x();
				double q13 = current_localization.trans().y();
				double q23 = current_localization.trans().z();

				vector<double> temp_path{q00, q01, q02, q03, q10, q11, q12, q13, q20, q21, q22, q23};
				kitti_path.push_back(temp_path);
			#endif


			current_temp_localization = current_localization;
			pcl::fromROSMsg(*laser, *laserCloudIn);

			transformPointsCloud2Frame(laserCloudIn, query, current_localization);
			InsertPoints(query);

			//ISAM初始化
			gtsam::Vector3 translation(current_localization.trans().x(), current_localization.trans().y(), current_localization.trans().z());
			gtsam::Rot3 rotation(gtsam::Rot3::RzRyRx(current_localization.rot().toEuler().x(), current_localization.rot().toEuler().y(), current_localization.rot().toEuler().z()));
			gtsam::Pose3 pose(rotation, translation);

			gtsam::Vector6 noise;
			noise << 0.1, 0.1, 0.1, 0.02, 0.02, 0.02;
			gtsam::noiseModel::Diagonal::shared_ptr covariance = gtsam::noiseModel::Diagonal::Variances(noise);

			gtsam::NonlinearFactorGraph new_factor;
			gtsam::Values new_value;
			new_factor.add(gtsam::PriorFactor<gtsam::Pose3>(key, pose, covariance));
			new_value.insert(key, pose);

			isam->update(new_factor, new_value);
			values = isam->calculateEstimate();
			key++;

			odometry = gtsam::Pose3::identity();

			#ifdef LOG_FLAG
				cout<<"LaserMapping ----->  Initial done!"<<endl;//以秒为单位显示之
			#endif

			is_initial = true;
			return;
		}

		update_count ++;
		//当前里程计预处理
		last_odom = current_odom;
		current_odom = Pose6D(Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z)
				, Quaternion(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z));
		incremental_odom = last_odom.inv() * current_odom;
		current_localization = current_localization* incremental_odom;

		if(update_count == 2){  //降频mapping

			#ifdef LOG_FLAG
				double  start, finish;
				start = clock();//取开始时间
			#endif

			update_count = 0;
			//当前点云预处理
			pcl::fromROSMsg(*laser, *laserCloudIn);
			transformPointsCloud2Frame(laserCloudIn, query, current_localization);//query是根据当前定位位姿转换到map下的
			
			//找到map中最近点云
			ApproxNearestNeighbors(query, reference);

			//转换得到用于定位匹配的query_base，reference_base
			pcl::PointCloud<pcl::PointXYZ>::Ptr query_base(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr reference_base(new pcl::PointCloud<pcl::PointXYZ>());
			copyPointCloud(*laserCloudIn, *query_base);
			transformPointsCloud2Frame(reference, reference_base, current_localization.inv());

			//scan-to-map matching
			float fitnessScore;
			Pose6D poseUpdate = updateLocalization(query_base, reference_base, fitnessScore);

			//更新上次定位位姿、当前定位位姿、增量定位位姿
			last_localization = current_temp_localization;
			current_localization = current_localization * poseUpdate;
			//平面约束
			//current_localization = Pose6D(Vector3(current_localization.trans().x(), current_localization.trans().y(), 0.0), Quaternion(0.0, 0.0, current_localization.rot().toEuler().z()));
			
			current_temp_localization = current_localization;
			incremental_localization =  last_localization.inv() * current_localization;

			//ISAM更新

			gtsam::NonlinearFactorGraph new_factor;
			gtsam::Values new_value;

			gtsam::Vector6 noise; noise << fitnessScore, fitnessScore, fitnessScore, fitnessScore, fitnessScore, fitnessScore;
			gtsam::noiseModel::Diagonal::shared_ptr covariance = gtsam::noiseModel::Diagonal::Variances(noise);
			gtsam::Pose3 new_odometry = ToGtsam(incremental_localization);
			new_factor.add(gtsam::BetweenFactor<gtsam::Pose3>(key-1, key, new_odometry, covariance));

			gtsam::Pose3 last_pose = values.at<gtsam::Pose3>(key-1);
			new_value.insert(key, last_pose.compose(new_odometry));

			isam->update(new_factor, new_value);
			values = isam->calculateEstimate();
			int current_pose_key = key++;

			
			//是否是关键帧
			if(check_for_loop_closure && isKeyFrame(new_odometry)){

				#ifdef LOG_FLAG
					cout<<"LaserMapping ----->  key = "<<key<<endl;//以秒为单位显示之
				#endif
				pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_in_copy(new pcl::PointCloud<pcl::PointXYZ>());
				copyPointCloud(*laserCloudIn, *laser_cloud_in_copy);
				keyed_scans.insert(std::pair<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>(current_pose_key, laser_cloud_in_copy));
				int closure_key;
				if(findLoopClosure(current_pose_key, closure_key)){
					#ifdef LOG_FLAG
						cout<<"LaserMapping ----->  Loop closure detected: between "<<current_pose_key<<" and "<< closure_key<<endl;
					#endif
					// 发现闭环，重新产生点云地图
					pcl::PointCloud<pcl::PointXYZ>::Ptr regenerated_map(new pcl::PointCloud<pcl::PointXYZ>());
					GetMaximumLikelihoodPoints(regenerated_map);

					map_data.reset(new pcl::PointCloud<pcl::PointXYZ>());
					map_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(map_octree_resolution));
					map_octree->setInputCloud(map_data);
					current_localization = FromGtsam(values.at<gtsam::Pose3>(current_pose_key));//重设置定位位姿

					pcl::PointCloud<pcl::PointXYZ>::Ptr unused(new pcl::PointCloud<pcl::PointXYZ>);
					InsertPoints(regenerated_map);
				}
				else{
					//添加到地图中
					transformPointsCloud2Frame(laserCloudIn, aftMappingCloud, current_localization);
					InsertPoints(aftMappingCloud);
				}
			}
			else{
				//添加到地图中
				transformPointsCloud2Frame(laserCloudIn, aftMappingCloud, current_localization);
				InsertPoints(aftMappingCloud);
				
			}
			pubMapData();
			pubLowerFreq();
			
			#ifdef LOG_FLAG
				finish = clock();//取结束时间
				cout<<"LaserMapping ----->  Time consuming "<<(finish - start) / CLOCKS_PER_SEC<< " seconds"<<endl;//以秒为单位显示
			#endif
			
		}
		#ifdef KITTI_PATH
			vector<double> temp(9,0.0);
			current_localization.rot().toRotMatrix(temp);
			double q00 = temp[0]; double q01 = temp[1]; double q02 = temp[2];
			double q10 = temp[3]; double q11 = temp[4]; double q12 = temp[5];
			double q20 = temp[6]; double q21 = temp[7]; double q22 = temp[8];

			double q03 = current_localization.trans().x();
			double q13 = current_localization.trans().y();
			double q23 = current_localization.trans().z();

			vector<double> temp_path{q00, q01, q02, q03, q10, q11, q12, q13, q20, q21, q22, q23};
			kitti_path.push_back(temp_path);
		#endif
		pubHigherFreq();

	}
	void GetMaximumLikelihoodPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
	{
		//图优化之后，将所有激光帧按照优化后的位姿重新计算
		for (const auto& keyed_pose : values)
		{
			const unsigned int key_index = keyed_pose.key;

			// 是否是关键帧
			if (!keyed_scans.count(key_index))
				continue;

			Pose6D pose = FromGtsam(values.at<gtsam::Pose3>(key_index));
			Eigen::Matrix4d b2w;

			Eigen::Matrix<double, 3, 3> R;
			Eigen::Matrix<double, 3, 1> T;

			vector<double> temp(9,0.0);
			pose.rot().toRotMatrix(temp);
			R(0,0) = temp[0]; R(0,1) = temp[1]; R(0,2) = temp[2];
			R(1,0) = temp[3]; R(1,1) = temp[4]; R(1,2) = temp[5];
			R(2,0) = temp[6]; R(2,1) = temp[7]; R(2,2) = temp[8];

			T(0,0) = pose.trans().x();
			T(1,0) = pose.trans().y();
			T(2,0) = pose.trans().z();

			b2w.block(0, 0, 3, 3) = R;
			b2w.block(0, 3, 3, 1) = T;

			// 转换到世界坐标系
			pcl::PointCloud<pcl::PointXYZ> scan_world;
			pcl::transformPointCloud(*keyed_scans[key_index], scan_world, b2w);

			// 添加到最终的输出
			*points += scan_world;
		}
	}
	bool findLoopClosure(int current_key, int& closure_key){

		// 刚闭环结束之后会经过一段时间再考虑闭环
		if (std::fabs(current_key - last_closure_key) < poses_before_reclosing)
			return false;

		const Pose6D pose1 = FromGtsam(values.at<gtsam::Pose3>(current_key));
		const pcl::PointCloud<pcl::PointXYZ>::Ptr scan1 = keyed_scans[current_key];

		// 遍历所有之前位姿，找到与当前位姿最匹配的一个（闭环）
		bool closed_loop = false;
		float fitnessMinResult= max_tolerable_fitness; //返回最匹配的一帧
		Pose6D deltaResult; //两帧之间的位姿变换
		int keyResut =0; //找到的最匹配的key
	  	for (const auto& keyed_pose : values) 
	  	{
	    		const int other_key = keyed_pose.key;
	    		//自己不考虑
			if (other_key == current_key)
				continue;
			// 最近几帧不考虑
			if (std::fabs(current_key - other_key) < skip_recent_poses)
				continue;
			// 不是关键帧不考虑
			if (!keyed_scans.count(other_key))
				continue;

	    		Pose6D pose2 = FromGtsam(values.at<gtsam::Pose3>(other_key));
	    		Pose6D distance_pose =pose1.inv() * pose2;

			//proximity_threshold_表示一个半径为这么大的球体，比较是否落在球里面，若是则后续ICP判断
			if (distance_pose.trans().norm() < proximity_threshold) 
			{
				#ifdef LOG_FLAG
					cout<<"LaserMapping ----->  Find pose in proximity_threshold!"<<endl;
				#endif

				const pcl::PointCloud<pcl::PointXYZ>::Ptr scan2 = keyed_scans[other_key];
				Pose6D deltaTemp;
				float fitnessTemp = 0.0; 

				if (performICP(scan1, scan2,  deltaTemp, fitnessTemp)) 
				{
					#ifdef LOG_FLAG
						cout<<"LaserMapping ----->  performICP fitnessTemp = "<<fitnessTemp<<endl;
					#endif
					// 找到一个闭环位姿
					if(fitnessTemp < fitnessMinResult)
					{
						fitnessMinResult = fitnessTemp;
						keyResut = other_key;
						deltaResult = deltaTemp;
						closed_loop = true;
					}
				}
			}
		}
		if(closed_loop)
		{
			#ifdef LOG_FLAG
				cout<<"LaserMapping ----->  Closure detected with fitnessScore = "<<fitnessMinResult<<endl;
			#endif
			gtsam::NonlinearFactorGraph new_factor;

			gtsam::Vector6 noise; 
			noise << fitnessMinResult, fitnessMinResult, fitnessMinResult, fitnessMinResult, fitnessMinResult, fitnessMinResult;
			gtsam::noiseModel::Diagonal::shared_ptr covariance = gtsam::noiseModel::Diagonal::Variances(noise);

			new_factor.add(gtsam::BetweenFactor<gtsam::Pose3>(current_key, keyResut, ToGtsam(deltaResult), covariance));
			isam->update(new_factor, gtsam::Values());

			last_closure_key = current_key;
			closure_key = keyResut;
		}
		values = isam->calculateEstimate();
		return closed_loop;
	}

	bool performICP(pcl::PointCloud<pcl::PointXYZ>::Ptr query_base, pcl::PointCloud<pcl::PointXYZ>::Ptr reference_base, Pose6D& delta, float& fitnessScore){

		//scan-to-map matching--------->>>>>point-to-plane ICP
		pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud(*query_base, *src);
		pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud(*reference_base, *tgt);

		pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
		norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
		norm_est.setKSearch(50);
		norm_est.setInputCloud(tgt);
		norm_est.compute(*tgt);

		pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
		typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
		boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
		icp.setTransformationEstimation(point_to_plane); // key

		icp.setInputSource(src);
		icp.setInputTarget(tgt);

		icp.setRANSACIterations(30);
		icp.setMaximumIterations(60);
		icp.setTransformationEpsilon(1e-6);
		icp.setMaxCorrespondenceDistance(0.3); //点对的最大距离超过0.1则忽略!!!!!!
		pcl::PointCloud<pcl::PointNormal> output;
		icp.align(output); // align 的另一个重载可以设置一个初始矩阵guess

		if(!icp.hasConverged())
			return false;

		const Eigen::Matrix4f T = icp.getFinalTransformation();
		fitnessScore = icp.getFitnessScore();

		Pose6D poseUpdate( Vector3(T(0, 3), T(1, 3), T(2, 3)), 
			Quaternion(T(0, 0), T(0, 1), T(0, 2),
			T(1, 0), T(1, 1), T(1, 2),
			T(2, 0), T(2, 1), T(2, 2)));
		delta = poseUpdate.inv();
		//平面约束
		//delta = Pose6D(Vector3(delta.trans().x(), delta.trans().y(), 0.0), Quaternion(0.0, 0.0, delta.rot().toEuler().z()));
			
		return true;
	}

	bool isKeyFrame(gtsam::Pose3& new_odometry)
	{
		odometry= odometry.compose(new_odometry);
		if (odometry.translation().norm() > translation_threshold) 
		{
			odometry = gtsam::Pose3::identity();
			return true;
		}
		return false;
	}

	gtsam::Pose3 ToGtsam(const Pose6D& pose){
		gtsam::Vector3 translation(pose.trans().x(), pose.trans().y(), pose.trans().z());
		gtsam::Rot3 rotation(gtsam::Rot3::RzRyRx(pose.rot().toEuler().x(), pose.rot().toEuler().y(), pose.rot().toEuler().z()));
		return gtsam::Pose3(rotation, translation);
	}

	Pose6D FromGtsam(const gtsam::Pose3& pose){
		Vector3 v(pose.translation().x(), pose.translation().y(), pose.translation().z());
		Quaternion q(pose.rotation().roll(), pose.rotation().pitch(), pose.rotation().yaw());
		return Pose6D(v, q);
	}

	Pose6D updateLocalization(pcl::PointCloud<pcl::PointXYZ>::Ptr query_base, pcl::PointCloud<pcl::PointXYZ>::Ptr reference_base, float& fitnessScore){

		//scan-to-map matching--------->>>>>point-to-plane ICP
		pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud(*query_base, *src);
		pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
		pcl::copyPointCloud(*reference_base, *tgt);

		pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
		norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
		norm_est.setKSearch(50);
		norm_est.setInputCloud(tgt);
		norm_est.compute(*tgt);

		pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
		typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
		boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
		icp.setTransformationEstimation(point_to_plane); // key

		icp.setInputSource(src);
		icp.setInputTarget(tgt);

		icp.setRANSACIterations(50);
		icp.setMaximumIterations(100);
		icp.setTransformationEpsilon(1e-6);
		icp.setMaxCorrespondenceDistance(0.2); //点对的最大距离超过0.1则忽略!!!!!!
		pcl::PointCloud<pcl::PointNormal> output;
		icp.align(output); // align 的另一个重载可以设置一个初始矩阵guess
		const Eigen::Matrix4f T = icp.getFinalTransformation();
		fitnessScore = icp.getFitnessScore();

		Pose6D poseUpdate( Vector3(T(0, 3), T(1, 3), T(2, 3)), 
			Quaternion(T(0, 0), T(0, 1), T(0, 2),
			T(1, 0), T(1, 1), T(1, 2),
			T(2, 0), T(2, 1), T(2, 2)));
		return poseUpdate;
	}
	//转换到地图坐标系
	void transformPointsCloud2Frame(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr tgt, Pose6D t){

		Eigen::Matrix<double, 3, 3> R;
		Eigen::Matrix<double, 3, 1> T;

		vector<double> temp(9,0.0);
		t.rot().toRotMatrix(temp);
		R(0,0) = temp[0]; R(0,1) = temp[1]; R(0,2) = temp[2];
		R(1,0) = temp[3]; R(1,1) = temp[4]; R(1,2) = temp[5];
		R(2,0) = temp[6]; R(2,1) = temp[7]; R(2,2) = temp[8];

		T(0,0) = t.trans().x();
		T(1,0) = t.trans().y();
		T(2,0) = t.trans().z();

		Eigen::Matrix4d tf;
		tf.block(0, 0, 3, 3) = R;
		tf.block(0, 3, 3, 1) = T;

		pcl::transformPointCloud(*src, *tgt, tf);
	}
	void InsertPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
		for (int i = 0; i < cloud->points.size(); ++i) 
		{
			const pcl::PointXYZ p = cloud->points[i];
			if (!map_octree->isVoxelOccupiedAtPoint(p)) 
			{
				map_octree->addPointToCloud(p, map_data);
			}
		}
	}
	void ApproxNearestNeighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr que, pcl::PointCloud<pcl::PointXYZ>::Ptr ref){
		ref->clear();
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr ref_octree;
		ref_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(ref_octree_resolution));
		ref_octree->setInputCloud(ref);
		//根据八叉树半径内最近点
		for (int i = 0; i < que->points.size(); ++i) 
		{
			vector<int> result_index;
			vector<float> unused;
			map_octree->radiusSearch(que->points[i], map_octree_radius, result_index, unused);
			for(int j = 0; j< result_index.size(); j++){

				if (!ref_octree->isVoxelOccupiedAtPoint(map_data->points[result_index[j]])) 
				{
					ref_octree->addPointToCloud(map_data->points[result_index[j]], ref);
				}
			}
		}
	}
	void pubMapData(){
		//发布激光定位点云数据
		pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*map_data, *map_filtered);
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setLeafSize(0.2, 0.2, 0.2);
		grid.setInputCloud(map_filtered);
		grid.filter(*map_filtered);

		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg(*map_filtered, cloud);
		cloud.header.stamp = stamp;
		cloud.header.frame_id = fix_frame_id;
		pubMapCloud.publish(cloud);
	}
	void pubLowerFreq(){
		if (pubGraphNode.getNumSubscribers() > 0) 
		{
			visualization_msgs::Marker m;
			m.header.frame_id = fix_frame_id;
			m.ns = fix_frame_id;
			m.id = 2;
			m.action = visualization_msgs::Marker::ADD;
			m.type = visualization_msgs::Marker::SPHERE_LIST;
			m.color.r = 1.0;
			m.color.g = 0.0;
			m.color.b = 0.2;
			m.color.a = 0.8;
			m.scale.x = 0.8;  
			m.scale.y = 0.8;
			m.scale.z = 0.8;

			for (const auto& keyed_pose : values) 
			{
				geometry_msgs::Point msg;
				msg.x = values.at<gtsam::Pose3>(keyed_pose.key).translation().x();
				msg.y = values.at<gtsam::Pose3>(keyed_pose.key).translation().y();
				msg.z = values.at<gtsam::Pose3>(keyed_pose.key).translation().z();
				m.points.push_back(msg);
			}
			pubGraphNode.publish(m);
		}

	}
	void pubHigherFreq()
	{
		//发布激光定位点云数据
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg(*laserCloudIn, cloud);
		cloud.header.stamp = stamp;
		cloud.header.frame_id = base_frame_id;
		pubLaserLocalizerCloud.publish(cloud);
	
		//发布定位
		nav_msgs::Odometry laserLocalizer;
		laserLocalizer.header.frame_id = fix_frame_id;
		laserLocalizer.child_frame_id = base_frame_id;
		laserLocalizer.header.stamp = stamp;
		laserLocalizer.pose.pose.orientation.x = current_localization.rot().x();
		laserLocalizer.pose.pose.orientation.y = current_localization.rot().y();
		laserLocalizer.pose.pose.orientation.z = current_localization.rot().z();
		laserLocalizer.pose.pose.orientation.w = current_localization.rot().u();
		laserLocalizer.pose.pose.position.x = current_localization.x();
		laserLocalizer.pose.pose.position.y = current_localization.y();
		laserLocalizer.pose.pose.position.z = current_localization.z();
		pubLaserLocalizer.publish(laserLocalizer);

		//TF变换
		geometry_msgs::TransformStamped tf;
		geometry_msgs::Vector3 vec;
		vec.x = current_localization.x();
		vec.y = current_localization.y();
		vec.z = current_localization.z();
		geometry_msgs::Quaternion quat;
		quat.w = current_localization.rot().u();
		quat.x = current_localization.rot().x();
		quat.y = current_localization.rot().y();
		quat.z = current_localization.rot().z();
		geometry_msgs::Transform trans; 
		trans.translation = vec;
		trans.rotation = quat;
		tf.transform = trans;
		tf.header.stamp = stamp;
		tf.header.frame_id = fix_frame_id;
		tf.child_frame_id = base_frame_id;
		tfbr.sendTransform(tf);
	}

	void SerialPath2File(string posePath){
		std::ofstream pose_file; //输出位姿到文件
		pose_file.open(posePath.c_str(),std::ofstream::out | std::ofstream::app);

		for(int i=0; i<kitti_path.size();i++){
			for(int j=0; j< kitti_path[i].size()-1; j++){
				pose_file<<kitti_path[i][j]<<" ";
			}
			pose_file<<kitti_path[i][kitti_path[i].size()-1]<<endl;
		}
		pose_file.close();

	}
};



int main(int argc, char** argv){
	
	ros::init(argc, argv, "LaserMapping");
	ros::NodeHandle n;

	ROS_INFO("\033[1;32m---->\033[0m LaserMapping Started.");
	LaserMapping lm(n);
	ros::spin();
	return 0;
}