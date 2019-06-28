#include "mim_slam/util.h"

//#define LOG_FLAG
//#define SEG_PCD

int RGB_table[10][3]= { 	{255,255,0}, //黄色
						{255,97,0},  //橙色	
						{245,222,197}, //淡黄色
						{255,235,205}, //白杏仁	
						{0,0,255}, //天蓝色
						{65, 105,225}, //品蓝
						{0,255,255}, // 青色
		         			{8,46,84}, //靛青
		         			{0,255,0}, //绿色
		         			{255,0,0} //红色
					  };
class StablePointExtract{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input;
	pcl::PointCloud<pcl::PointXYZ>::Ptr fullCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr outlierCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmented2Cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedCloudVec;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedRGB;

	vector<vector<float>> rangeMat;
	vector<vector<int>> labelMat;
	vector<vector<int>> groundMat;
	int labelCount;
	pcl::PointXYZ nanPoint;

	//BFS队列
	int *allPushedIndX;
	int *allPushedIndY;

	int *queueIndX;
	int *queueIndY;
	std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

	void initialParam(){
		nanPoint.x = std::numeric_limits<float>::quiet_NaN();
		nanPoint.y = std::numeric_limits<float>::quiet_NaN();
		nanPoint.z = std::numeric_limits<float>::quiet_NaN();

		cloud_input.reset(new pcl::PointCloud<pcl::PointXYZ>());
		fullCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		groundCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		outlierCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		outputCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		segmentedCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		segmented2Cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		segmentedRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
		
		cloud_input->clear();
		groundCloud->clear();
		outlierCloud->clear();
		segmentedCloud->clear();
		segmented2Cloud->clear();
		outputCloud->clear();
		fullCloud->points.resize(N_SCAN*Horizon_SCAN);

		//激光点深度, range_max表示没有返回激光数据
		rangeMat.resize(N_SCAN, vector<float>(Horizon_SCAN, range_max));
		labelMat.resize(N_SCAN, vector<int>(Horizon_SCAN, 0));
		//-1表示该方向为NAN点，0表示不是地面，1表示是地面
		groundMat.resize(N_SCAN, vector<int>(Horizon_SCAN, 0));
		labelCount =1;

		allPushedIndX = new int[N_SCAN*Horizon_SCAN];
		allPushedIndY = new int[N_SCAN*Horizon_SCAN];

		queueIndX = new int[N_SCAN*Horizon_SCAN];
		queueIndY = new int[N_SCAN*Horizon_SCAN];

		std::pair<int8_t, int8_t> neighbor;
		neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
		neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
		neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
		neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

		std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
	}
	void projectPointCloud(){
		float verticalAngle, horizonAngle, range;
		int rowIdn, columnIdn, cloudSize, index; 
		pcl::PointXYZ thisPoint;

		cloudSize = cloud_input->points.size();
		#ifdef LOG_FLAG
			cout<<"StablePointExtract ----->  projectPointCloud: cloud_input cloudSize = "<<cloudSize<<endl;
		#endif
		int not_max_point =0;
		for (size_t i = 0; i < cloudSize; ++i){
			thisPoint.x = cloud_input->points[i].x;
			thisPoint.y = cloud_input->points[i].y;
			thisPoint.z = cloud_input->points[i].z;
			//垂直角度，转换成行坐标：-15度对应0行，+15度对应第15行
			verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
			rowIdn = int((verticalAngle + ang_bottom) / ang_res_y + 0.5);
			
			if (rowIdn < 0 || rowIdn >= N_SCAN)
				continue;
			//水平角度
			horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

			//转换成列坐标：-90度对应0，(-180,+180)对应500， 90度对应1000, 0度对应1500， -89.8对应1999
			if (horizonAngle <= -90)
				columnIdn = -int(horizonAngle / ang_res_x +0.5) - Horizon_SCAN/4; 
			else if (horizonAngle >= 0)
				columnIdn = -int(horizonAngle / ang_res_x+0.5) + Horizon_SCAN*3/4;
			else
				columnIdn = Horizon_SCAN*3/4 - int(horizonAngle / ang_res_x+0.5);
			if(columnIdn<0 || columnIdn>= Horizon_SCAN)
				continue;

			range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

			rangeMat[rowIdn][columnIdn] = range;
			index = columnIdn  + rowIdn * Horizon_SCAN;
			fullCloud->points[index] = thisPoint;

		}

	}
	void groundRemoval(){
		int lowerInd, mediumInd, upperInd;
		float diffX, diffY, diffZ, angle;

		for (int j = 0; j < Horizon_SCAN; ++j){
			for (int i = 1; i < groundScanInd; ++i){
				//点为NAN不作处理
				if (rangeMat[i-1][j]==range_max || rangeMat[i][j]==range_max ){
					groundMat[i][j] = -1;
					continue;
				}
				lowerInd = j + ( i-1 )*Horizon_SCAN;
				mediumInd =  j + ( i )*Horizon_SCAN;

				diffX = fullCloud->points[mediumInd].x - fullCloud->points[lowerInd].x;
				diffY = fullCloud->points[mediumInd].y - fullCloud->points[lowerInd].y;
				diffZ= fullCloud->points[mediumInd].z - fullCloud->points[lowerInd].z;
				angle = atan2(diffZ, sqrt(diffX*diffX+ diffY*diffY) ) * 180 / M_PI;
				//垂直角度很小时，认为是地面点
				if (fabs(angle - sensorMountAngle) <= groundRemoveThre){
					groundMat[i-1][j] = 1;
					groundMat[i][j] = 1;
				}
			}
		}

		//为地面点或NAN点时不考虑为segment点
		for (int i = 0; i < N_SCAN; ++i){
			for (int j = 0; j < Horizon_SCAN; ++j){
				if (groundMat[i][j] == 1 || rangeMat[i][j] == range_max){
					labelMat[i][j] = -1;
				}
			}
		}
		//生成地面点云
		for (int i = 0; i <= groundScanInd; ++i){
			for (int j = 0; j < Horizon_SCAN; ++j){
				if (groundMat[i][j] == 1)
					groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
			}
		}
		#ifdef LOG_FLAG
			cout<<"StablePointExtract ----->  ground point num: groundcloudNum = "<<groundCloud->size()<<endl;
		#endif
	}
	//BFS深搜，参考论文：Fast Range Image-based Segmentation...
	void labelComponents(int row, int col){
		float d1, d2, alpha, angle;
		int fromIndX, fromIndY, thisIndX, thisIndY; 
		bool lineCountFlag[N_SCAN] = {false};

		queueIndX[0] = row;
		queueIndY[0] = col;
		int queueSize = 1;
		int queueStartInd = 0;
		int queueEndInd = 1;

		allPushedIndX[0] = row;
		allPushedIndY[0] = col;
		int allPushedIndSize = 1;

		while(queueSize > 0){
			fromIndX = queueIndX[queueStartInd];
			fromIndY = queueIndY[queueStartInd];
			--queueSize;
			++queueStartInd;
			labelMat[fromIndX][fromIndY] = labelCount;

			for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){

				thisIndX = fromIndX + (*iter).first;
				thisIndY = fromIndY + (*iter).second;

				if (thisIndX < 0 || thisIndX >= N_SCAN)
					continue;

				if (thisIndY < 0)
					thisIndY = Horizon_SCAN - 1;
				if (thisIndY >= Horizon_SCAN)
					thisIndY = 0;
				//说明已经被决策过了
				if (labelMat[thisIndX][thisIndY] != 0)
					continue;

				d1 = std::max(rangeMat[fromIndX][fromIndY], rangeMat[thisIndX][thisIndY]);
				d2 = std::min(rangeMat[fromIndX][fromIndY], rangeMat[thisIndX][thisIndY]);

				if ((*iter).first == 0)
					alpha = segmentAlphaX;
				else if((*iter).second == 0)
					alpha = segmentAlphaY;

				angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));//论文中的beta

				if (angle > segmentTheta){
					queueIndX[queueEndInd] = thisIndX;
					queueIndY[queueEndInd] = thisIndY;
					++queueSize;
					++queueEndInd;

					labelMat[thisIndX][thisIndY]= labelCount;//标记该激光点为第几号segment
					lineCountFlag[thisIndX] = true; //标记占据该条扫描线
					//存储，为当前segment的点
					allPushedIndX[allPushedIndSize] = thisIndX;
					allPushedIndY[allPushedIndSize] = thisIndY;
					++allPushedIndSize;
				}
			}
		}


		bool feasibleSegment = false;
		//如果所有点数大于segmentMinNum，则说明是一个segment
		if (allPushedIndSize >= segmentMinNum)
			feasibleSegment = true;
		//否则如果点数大于segmentValidPointNum，且占据3条扫描线也是一个segment
		else if (allPushedIndSize >= segmentValidPointNum){
			int lineCount = 0;
			for (int i = 0; i < N_SCAN; ++i)
				if (lineCountFlag[i] == true)
					++lineCount;
				if (lineCount >= segmentValidLineNum)
					feasibleSegment = true;            
		}
		//若是segment，则labelCount++，便于下次使用
		if (feasibleSegment == true){
			++labelCount;
		}else{  //否则标记赋值999999，其为异常点
			for (int i = 0; i < allPushedIndSize; ++i){
				labelMat[allPushedIndX[i]][allPushedIndY[i]] = 999999;
			}
		}
	}
	/*
	* labelMat[i][j]取值：
	* 	-1：地面或NAN点；
	*	0：默认初始值
	* 	1～val：为该激光点所属于的segment标号
	*	999999：异常点
	*/
	void cloudSegmentation(){
		for (int i = 0; i < N_SCAN; ++i)
			for (int j = 0; j < Horizon_SCAN; ++j)
				if (labelMat[i][j] == 0)
					labelComponents(i, j);
		#ifdef LOG_FLAG
			cout<<"StablePointExtract ----->  cloudSegmentation: labelCount = "<<labelCount<<endl;
		#endif

		segmentedCloudVec.resize(labelCount-1);
		for(int i=0;i<segmentedCloudVec.size();i++){
			segmentedCloudVec[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
		}

		for (int i = 0; i < N_SCAN; ++i) {
			for (int j = 0; j < Horizon_SCAN; ++j) {
				if (labelMat[i][j] > 0 ){
					if (labelMat[i][j] == 999999) //异常点
						outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
					else{  //segment点
						segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
						segmentedCloudVec[labelMat[i][j]-1]->push_back(fullCloud->points[j + i*Horizon_SCAN]);
					}

				}
			}
		}
		#ifdef LOG_FLAG
			cout<<"StablePointExtract ----->  segmented point num: segmentedCloudNum = "<<segmentedCloud->size()<<endl;
			cout<<"StablePointExtract ----->  outlier point num: outlierCloudNum = "<<outlierCloud->size()<<endl;
		#endif
	}
	void preSegmentHandle(){
		for(int i=0; i< segmentedCloudVec.size();i++){
			Eigen::Matrix3f covariance_matrix;
			Eigen::Vector4f xyz_centroid;
			Eigen::Vector3f eigen_val;
			computeMeanAndCovarianceMatrix (*segmentedCloudVec[i], covariance_matrix, xyz_centroid);
			pcl::eigen33 (covariance_matrix, eigen_val);
			float curvature = eigen_val[0] / (eigen_val[0]+eigen_val[1]+eigen_val[2]);
			#ifdef LOG_FLAG
				//cout<<"StablePointExtract ----->  segmentedCloudVec["<<i<<"]  eigen_val = "<<eigen_val[0]<<","<<eigen_val[1]<<","<<eigen_val[2]<<endl;
				//cout<<"StablePointExtract ----->   segmentedCloudVec["<<i<<"]  curvature = "<<curvature<<endl;
			#endif
			if(curvature < 0.001)
				*segmented2Cloud += *segmentedCloudVec[i];

		}
	}
	void colorSegment(){
		for(int i=0; i< segmentedCloudVec.size();i++){
			int index = i%10;
			for(int j=0;j<segmentedCloudVec[i]->size();j++){
				pcl::PointXYZRGB p;
				p.x = segmentedCloudVec[i]->points[j].x;
				p.y = segmentedCloudVec[i]->points[j].y;
				p.z = segmentedCloudVec[i]->points[j].z;
				p.r = RGB_table[index][0];
				p.g = RGB_table[index][1];
				p.b = RGB_table[index][2];
				segmentedRGB->push_back(p);
			}
		}
		pcl::PCDWriter writer;  
		writer.write("/home/zhaoming/bagFile/segment_pcd/seg.pcd", *segmentedRGB); 
	}
	void downSample(){
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_filtered(new pcl::PointCloud<pcl::PointXYZ>());

		//半径异常点滤波
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> rad;
		rad.setInputCloud(groundCloud);
		rad.setRadiusSearch(0.5);
		rad.setMinNeighborsInRadius(10);
		rad.filter(*ground_filtered);

		// rad.setInputCloud(segmentedCloud);
		// rad.setRadiusSearch(0.3);
		// rad.setMinNeighborsInRadius(5);
		// rad.filter(*segmented_filtered);

		//体素栅格滤波
		double param_res = 0.2;
		pcl::VoxelGrid<pcl::PointXYZ> grid;
		grid.setLeafSize(param_res, param_res, param_res);
		grid.setInputCloud(ground_filtered);
		grid.filter(*ground_filtered);

		grid.setLeafSize(param_res, param_res, param_res);
		grid.setInputCloud(segmentedCloud);
		grid.filter(*segmented_filtered);

		grid.setLeafSize(param_res, param_res, param_res);
		grid.setInputCloud(segmented2Cloud);
		grid.filter(*segmented2Cloud);

		*outputCloud += *ground_filtered;
		*outputCloud += *segmented_filtered;
		#ifdef LOG_FLAG
			cout<<"StablePointExtract ----->  output point num: outputCloudNum = "<<outputCloud->size()<<endl;
		#endif
	}

public:
	StablePointExtract(){}
	~StablePointExtract(){
		if(allPushedIndX){
			delete [] allPushedIndX;
			allPushedIndX = NULL;
		}
		if(allPushedIndY){
			delete [] allPushedIndY;
			allPushedIndY = NULL;
		}	

		if(queueIndX){
			delete [] queueIndX;
			queueIndX = NULL;
		}		
		if(queueIndY){
			delete [] queueIndY;
			queueIndY = NULL;
		}
	}

	void processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
		#ifdef LOG_FLAG
			double  start, finish;
			start = clock();//取开始时间
		#endif
		initialParam();
		copyPointCloud(*cloud_in, *cloud_input);
		projectPointCloud();
		groundRemoval();
		cloudSegmentation();
		//preSegmentHandle();

		#ifdef SEG_PCD
			colorSegment();
		#endif

		downSample();
		#ifdef LOG_FLAG
			finish = clock();//取结束时间
			cout<<"StablePointExtract ----->  Time consuming "<<(finish - start) / CLOCKS_PER_SEC<< " seconds"<<endl;//以秒为单位显示之
		#endif
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getGroundCloud(){
		return groundCloud;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSegmentedCloud(){
		return segmentedCloud;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSegmented2Cloud(){
		return segmented2Cloud;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getOutlierCloud(){
		return outlierCloud;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getOutputCloud(){
		return outputCloud;
	}
};


ros::Publisher pubGroundCloud;
ros::Publisher pubSegmentedCloud;
ros::Publisher pubSegmented2Cloud;
ros::Publisher pubOutlierCloud;
ros::Publisher pubOutputCloud;

ros::Subscriber subLaserCloud;

void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn;
	laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
	std::vector<int> indices; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*laserCloudIn,*laserCloudIn, indices); //去除点云中的NaN点

	StablePointExtract sp;
	sp.processPointCloud(laserCloudIn);

	if (pubGroundCloud.getNumSubscribers() != 0){
		sensor_msgs::PointCloud2 laserCloudTemp;
		pcl::toROSMsg(*sp.getGroundCloud(), laserCloudTemp);
		laserCloudTemp.header.stamp = laserCloudMsg->header.stamp;
		laserCloudTemp.header.frame_id = laserCloudMsg->header.frame_id;
		pubGroundCloud.publish(laserCloudTemp);
	}
	if (pubSegmentedCloud.getNumSubscribers() != 0){
		sensor_msgs::PointCloud2 laserCloudTemp;
		pcl::toROSMsg(*sp.getSegmentedCloud(), laserCloudTemp);
		laserCloudTemp.header.stamp = laserCloudMsg->header.stamp;
		laserCloudTemp.header.frame_id = laserCloudMsg->header.frame_id;
		pubSegmentedCloud.publish(laserCloudTemp);
	}
	if (pubSegmented2Cloud.getNumSubscribers() != 0){
		sensor_msgs::PointCloud2 laserCloudTemp;
		pcl::toROSMsg(*sp.getSegmented2Cloud(), laserCloudTemp);
		laserCloudTemp.header.stamp = laserCloudMsg->header.stamp;
		laserCloudTemp.header.frame_id = laserCloudMsg->header.frame_id;
		pubSegmented2Cloud.publish(laserCloudTemp);
	}
	if (pubOutlierCloud.getNumSubscribers() != 0){
		sensor_msgs::PointCloud2 laserCloudTemp;
		pcl::toROSMsg(*sp.getOutlierCloud(), laserCloudTemp);
		laserCloudTemp.header.stamp = laserCloudMsg->header.stamp;
		laserCloudTemp.header.frame_id = laserCloudMsg->header.frame_id;
		pubOutlierCloud.publish(laserCloudTemp);
	}
	if (pubOutputCloud.getNumSubscribers() != 0){
		sensor_msgs::PointCloud2 laserCloudTemp;
		pcl::toROSMsg(*sp.getOutputCloud(), laserCloudTemp);
		laserCloudTemp.header.stamp = laserCloudMsg->header.stamp;
		laserCloudTemp.header.frame_id = laserCloudMsg->header.frame_id;
		pubOutputCloud.publish(laserCloudTemp);
	}
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "StablePointExtract");
	ros::NodeHandle nh;

	subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 10, &cloudHandler);
	pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 10);
	pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
	pubSegmented2Cloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented2_cloud", 1);
	pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
	pubOutputCloud = nh.advertise<sensor_msgs::PointCloud2> ("/output_cloud", 1);
	ROS_INFO("\033[1;32m---->\033[0m StablePoint Extract Started.");

	ros::spin();
	return 0;
}