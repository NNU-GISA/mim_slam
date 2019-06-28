#include "mim_slam/util.h"

//#define LOG_FLAG

class LaserOdometry{

private:
	bool is_initial;
	ros::Time last_cloud_stamp;
	ros::Time cur_cloud_stamp;

	bool imu_data_flag;
	bool wheel_odom_data_flag;
	bool lidar_data_flag;

	//imu、wheel_odom
	mutex m_mu;
	deque<sensor_msgs::Imu> imu_queue;
	deque<nav_msgs::Odometry> wheel_odom_queue;
	//icp的初始估计，使用imu
	Eigen::Matrix4d initial_guess;

	//用于匹配的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn;
	pcl::PointCloud<pcl::PointXYZ>::Ptr query;
	pcl::PointCloud<pcl::PointXYZ>::Ptr reference;

	Pose6D incremental_estimate; //默认均为0.
	Pose6D integrated_estimate;

	//发布、订阅
	ros::Subscriber subLaserCloud;
	ros::Subscriber subImu;
	ros::Subscriber subWheelOdom;

	tf2_ros::TransformBroadcaster tfbr;
	ros::Publisher pubLaserOdometry;
	ros::Publisher pubLaserOdometryCloud;

	bool getNearestImuIndex(ros::Time t, Pose6D& p){
		lock_guard<mutex> guard(m_mu);
		int min_index = -1;
		double min = std::numeric_limits<double>::max();
		for(int i=0; i< imu_queue.size();i++){
			if(fabs(imu_queue[i].header.stamp.toSec() - t.toSec()) < min){
				min = fabs(imu_queue[i].header.stamp.toSec() - t.toSec());
				min_index = i;
			}
		}
		#ifdef LOG_FLAG
			cout<<"LaserOdometry ----->  Time span between imu and lidar  "<<fabs(imu_queue[min_index].header.stamp.toSec() - t.toSec())<<endl;
		#endif
		if(min_index == -1){
			return false;
		}
		else{
			p.trans() = Vector3(0.0,0.0,0.0);
			p.rot() = Quaternion(imu_queue[min_index].orientation.w,
				imu_queue[min_index].orientation.x, imu_queue[min_index].orientation.y, imu_queue[min_index].orientation.z);
			return true;
		}

	}
	bool getNearestWheelOdomIndex(ros::Time t, Pose6D& p){
		lock_guard<mutex> guard(m_mu);
		int min_index = -1;
		double min = std::numeric_limits<double>::max();
		for(int i=0; i< wheel_odom_queue.size();i++){
			if(fabs(wheel_odom_queue[i].header.stamp.toSec() - t.toSec()) < min){
				min = fabs(wheel_odom_queue[i].header.stamp.toSec() - t.toSec());
				min_index = i;
			}
		}
		#ifdef LOG_FLAG
			cout<<"LaserOdometry ----->  Time span between wheel_odom and lidar = "<<fabs(wheel_odom_queue[min_index].header.stamp.toSec() - t.toSec())<<endl;
		#endif
		if(min_index == -1){
			return false;
		}
		else{
			p.trans() = Vector3(wheel_odom_queue[min_index].pose.pose.position.x, wheel_odom_queue[min_index].pose.pose.position.y, wheel_odom_queue[min_index].pose.pose.position.z);
			//p.trans() = Vector3(0.0,0.0,0.0);
			p.rot() = Quaternion(wheel_odom_queue[min_index].pose.pose.orientation.w,
				wheel_odom_queue[min_index].pose.pose.orientation.x, wheel_odom_queue[min_index].pose.pose.orientation.y, wheel_odom_queue[min_index].pose.pose.orientation.z);
			//p.rot() = Quaternion(0.0,0.0,0.0);
			return true;
		}

	}

public:
	LaserOdometry(ros::NodeHandle& n):is_initial(false), imu_data_flag(false), wheel_odom_data_flag(false), lidar_data_flag(false){

		laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZ>());
		query.reset(new pcl::PointCloud<pcl::PointXYZ>());
		reference.reset(new pcl::PointCloud<pcl::PointXYZ>());

		ros::NodeHandle nh(n);

		subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/output_cloud", 1, &LaserOdometry::cloudHandler, this);
		subImu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, &LaserOdometry::imuHandler, this);
		subWheelOdom = nh.subscribe<nav_msgs::Odometry>(wheel_odom_topic, 1, &LaserOdometry::wheelOdomHandler, this);

		pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom", 1, false);
		pubLaserOdometryCloud = nh.advertise<sensor_msgs::PointCloud2> ("/laser_odom_cloud", 1,false);

		initial_guess = Eigen::Matrix4d::Identity(4,4);
	}
	~LaserOdometry(){}

	void imuHandler(const sensor_msgs::ImuConstPtr& imuMsg){
		if(use_imu){
			lock_guard<mutex> guard(m_mu);
			if(imu_queue.size() <imu_queue_len){
				imu_queue.push_back(*imuMsg);
			}
			else{
				imu_queue.pop_front();
				imu_queue.push_back(*imuMsg);
			}
			imu_data_flag = true;
		}

	}
	void wheelOdomHandler(const nav_msgs::OdometryConstPtr& odomMsg){
		if(use_wheel_odom){
			lock_guard<mutex> guard(m_mu);
			if(wheel_odom_queue.size() <wheel_odom_queue_len){
				wheel_odom_queue.push_back(*odomMsg);
			}
			else{
				wheel_odom_queue.pop_front();
				wheel_odom_queue.push_back(*odomMsg);
			}
			wheel_odom_data_flag = true;
		}
	}

	void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

		last_cloud_stamp = cur_cloud_stamp;
		cur_cloud_stamp = laserCloudMsg->header.stamp;
		pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
		lidar_data_flag = true;
	}

	void updateOdometry(){

		if((use_imu && (imu_data_flag && lidar_data_flag  && imu_queue.back().header.stamp.toSec() >= cur_cloud_stamp.toSec() )) ||
			(use_wheel_odom && (wheel_odom_data_flag && lidar_data_flag &&  wheel_odom_queue.back().header.stamp.toSec() >= cur_cloud_stamp.toSec() )) ||
			(!use_imu && !use_wheel_odom && lidar_data_flag))
		{
			imu_data_flag = false;
			wheel_odom_data_flag = false;
			lidar_data_flag = false;

			#ifdef LOG_FLAG
				double  start, finish;
				start = clock();//取开始时间
				cout<<"LaserOdometry ----->  PointCloud: cloud_input cloudSize = "<<laserCloudIn->size()<<endl;
			#endif
			if(!is_initial){
				is_initial = true;
				copyPointCloud(*laserCloudIn, *query);
				return;
			}

			if(use_imu){
				Pose6D last_imu_pose;
				Pose6D cur_imu_pose; 

				if(getNearestImuIndex(last_cloud_stamp, last_imu_pose) && getNearestImuIndex(cur_cloud_stamp,cur_imu_pose)){
					Pose6D incre = last_imu_pose.inv() * cur_imu_pose;

					Eigen::Matrix<double, 3, 3> R;
					Eigen::Matrix<double, 3, 1> T;

					vector<double> temp(9,0.0);
					incre.rot().toRotMatrix(temp);
					R(0,0) = temp[0]; R(0,1) = temp[1]; R(0,2) = temp[2];
					R(1,0) = temp[3]; R(1,1) = temp[4]; R(1,2) = temp[5];
					R(2,0) = temp[6]; R(2,1) = temp[7]; R(2,2) = temp[8];

					T(0,0) = incre.trans().x();
					T(1,0) = incre.trans().y();
					T(2,0) = incre.trans().z();

					initial_guess.block(0, 0, 3, 3) = R;
					initial_guess.block(0, 3, 3, 1) = T;
				}		
			}
			if(use_wheel_odom){
				Pose6D last_wheel_dom_pose;
				Pose6D cur_wheel_odom_pose; 

				if(getNearestWheelOdomIndex(last_cloud_stamp, last_wheel_dom_pose) && getNearestWheelOdomIndex(cur_cloud_stamp,cur_wheel_odom_pose)){
					Pose6D incre = last_wheel_dom_pose.inv() * cur_wheel_odom_pose;

					Eigen::Matrix<double, 3, 3> R;
					Eigen::Matrix<double, 3, 1> T;

					vector<double> temp(9,0.0);
					incre.rot().toRotMatrix(temp);
					R(0,0) = temp[0]; R(0,1) = temp[1]; R(0,2) = temp[2];
					R(1,0) = temp[3]; R(1,1) = temp[4]; R(1,2) = temp[5];
					R(2,0) = temp[6]; R(2,1) = temp[7]; R(2,2) = temp[8];

					T(0,0) = incre.trans().x();
					T(1,0) = incre.trans().y();
					T(2,0) = incre.trans().z();

					initial_guess.block(0, 0, 3, 3) = R;
					initial_guess.block(0, 3, 3, 1) = T;
				}		
			}

			copyPointCloud(*query, *reference);
			copyPointCloud(*laserCloudIn, *query);

			//scan-to-scan matching--------->>>>>point-to-plane ICP
			pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
			pcl::copyPointCloud(*query, *src);
			pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
			pcl::copyPointCloud(*reference, *tgt);

			pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
			norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
			norm_est.setKSearch(30);
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
			icp.setMaxCorrespondenceDistance(0.2); //点对的最大距离超过0.1则忽略!!!!!!
			pcl::PointCloud<pcl::PointNormal> output;
			icp.align(output, initial_guess.cast<float>()); // align 的另一个重载可以设置一个初始矩阵guess
			const Eigen::Matrix4f T = icp.getFinalTransformation();

			incremental_estimate = Pose6D( Vector3(T(0, 3), T(1, 3), T(2, 3)), 
				Quaternion(T(0, 0), T(0, 1), T(0, 2),
				T(1, 0), T(1, 1), T(1, 2),
				T(2, 0), T(2, 1), T(2, 2)));

			integrated_estimate = integrated_estimate * incremental_estimate;
			//平面约束
			//integrated_estimate = Pose6D(Vector3(integrated_estimate.trans().x(), integrated_estimate.trans().y(), 0.0), Quaternion(0.0, 0.0, integrated_estimate.rot().toEuler().z()));

			pubForVisualizer();

			#ifdef LOG_FLAG
				//cout<<"LaserOdometry ----->  ICP fitnessScore = "<<icp.getFitnessScore()<<endl;
				//cout<<"LaserOdometry ----->  Current Pose: x= "<<integrated_estimate.x()<<", y = " <<integrated_estimate.y()
				//	<<" z = "<< integrated_estimate.z()<<endl;
			#endif

			#ifdef LOG_FLAG
				finish = clock();//取结束时间
				cout<<"LaserOdometry ----->  Time consuming"<<(finish - start) / CLOCKS_PER_SEC<< "seconds"<<endl;//以秒为单位显示之
			#endif
		}

	}
	void pubForVisualizer()
	{
		//发布激光里程计点云数据
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg(*laserCloudIn, cloud);
		cloud.header.stamp = cur_cloud_stamp;
		cloud.header.frame_id = odom_frame_id;
		pubLaserOdometryCloud.publish(cloud);
	
		//发布里程计
		nav_msgs::Odometry laserOdometry;
		laserOdometry.header.frame_id = fix_frame_id;
		laserOdometry.child_frame_id = odom_frame_id;
		laserOdometry.header.stamp = cur_cloud_stamp;
		laserOdometry.pose.pose.orientation.x = integrated_estimate.rot().x();
		laserOdometry.pose.pose.orientation.y = integrated_estimate.rot().y();
		laserOdometry.pose.pose.orientation.z = integrated_estimate.rot().z();
		laserOdometry.pose.pose.orientation.w = integrated_estimate.rot().u();
		laserOdometry.pose.pose.position.x = integrated_estimate.x();
		laserOdometry.pose.pose.position.y = integrated_estimate.y();
		laserOdometry.pose.pose.position.z = integrated_estimate.z();
		pubLaserOdometry.publish(laserOdometry);

		//TF变换
		geometry_msgs::TransformStamped tf;
		geometry_msgs::Vector3 vec;
		vec.x = integrated_estimate.x();
		vec.y = integrated_estimate.y();
		vec.z = integrated_estimate.z();
		geometry_msgs::Quaternion quat;
		quat.w = integrated_estimate.rot().u();
		quat.x = integrated_estimate.rot().x();
		quat.y = integrated_estimate.rot().y();
		quat.z = integrated_estimate.rot().z();
		geometry_msgs::Transform trans; 
		trans.translation = vec;
		trans.rotation = quat;
		tf.transform = trans;
		tf.header.stamp = cur_cloud_stamp;
		tf.header.frame_id = fix_frame_id;
		tf.child_frame_id = odom_frame_id;
		tfbr.sendTransform(tf);
	}

};



int main(int argc, char** argv){
	
	ros::init(argc, argv, "LaserOdometry");
	ros::NodeHandle n;

	ROS_INFO("\033[1;32m---->\033[0m LaserOdometry Started.");
	LaserOdometry lo(n);

	while (ros::ok())
	{
		lo.updateOdometry();
		ros::spinOnce();
	}
	return 0;
}