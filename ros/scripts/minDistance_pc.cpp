#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

using namespace::std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub_vis;

void filter_compute_distance_cloud(const sensor_msgs::PointCloud2ConstPtr& input){
	//Filter the point cloud, Downsampling and avoiding singularity

	pcl::PCLPointCloud2* cloud =new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;
	sensor_msgs::PointCloud2 output;

	pcl_conversions::toPCL(*input, *cloud); //convert to PCL data type but a ROS pcl data type called PointCloud2
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;// Apply filter
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize(0.01,0.01,0.01);
	sor.filter (cloud_filtered);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2 (cloud_filtered, *cloud_XYZ);

	//Compute the minDistance and publish marker of this point

	double minDistance=0;
	double maxDistance=1;
	double min_angle_radx=0.0;
	double min_angle_rady=0.0;
	double xX=0.0,yY=0.0,zZ=0.0;
	int count=0;

	for(size_t i = 0; i < cloud_XYZ->points.size(); i++){

		float pt_x = cloud_XYZ->points[i].x;
 		float pt_y = cloud_XYZ->points[i].y;
 		float pt_z = cloud_XYZ->points[i].z;

 		if(atan2(pt_z, pt_y)*(180/3.14159265358979323846)>80.00){

 			if(count == 0){
 				minDistance=hypot(pt_z, pt_x);
		      	min_angle_radx=atan2(pt_z,pt_x);
		      	min_angle_rady=atan2(pt_z, pt_y);
		      	xX=pt_x;
		      	yY=pt_y;
		      	zZ=pt_z;
		      	count++;

		      	visualization_msgs::Marker marker;
		     	marker.header.frame_id = "camera_depth_optical_frame";
		     	marker.header.stamp = ros::Time();
		      	marker.ns = "my_namespace";
		      	marker.id = 0;
		      	marker.type = visualization_msgs::Marker::SPHERE;
		      	marker.action = visualization_msgs::Marker::MODIFY;
		      	marker.pose.position.x = xX;
		      	marker.pose.position.y = yY;
		      	marker.pose.position.z = zZ;
		      	marker.pose.orientation.x = 0;
		      	marker.pose.orientation.y = 0;
		      	marker.pose.orientation.z = 0;
		      	marker.pose.orientation.w = 1.0;
		      	marker.scale.x = 0.05;
		      	marker.scale.y = 0.05;
		      	marker.scale.z = 0.05;
		      	marker.color.a = 1.0; 
		      	marker.color.r = 1.0;
		      	marker.color.g = 0.0;
		      	marker.color.b = 0.0;
		      	pub_vis.publish( marker );
		  		}
		  	else if(hypot(pt_z, pt_x)<minDistance){
		  		minDistance=hypot(pt_z, pt_x);
        		min_angle_radx=atan2(pt_z,pt_x);
		        min_angle_rady=atan2(pt_z, pt_y);
		        xX=pt_x;
		        yY=pt_y;
		        zZ=pt_z;
		        count++;

		        visualization_msgs::Marker marker;
		        marker.header.frame_id = "camera_depth_optical_frame";
		        marker.header.stamp = ros::Time();
		        marker.ns = "my_namespace";
		        marker.id = 0;
		        marker.type = visualization_msgs::Marker::SPHERE;
		        marker.action = visualization_msgs::Marker::MODIFY;
		        marker.pose.position.x = xX;
		        marker.pose.position.y = yY;
		        marker.pose.position.z = zZ;
		        marker.pose.orientation.x = 0;
		        marker.pose.orientation.y = 0;
		        marker.pose.orientation.z = 0;
		        marker.pose.orientation.w = 1.0;
		        marker.scale.x = 0.05;
		        marker.scale.y = 0.05;
		        marker.scale.z = 0.05;
		        marker.color.a = 1.0; 
		        marker.color.r = 1.0;
		        marker.color.g = 0.0;
		        marker.color.b = 0.0;
		        pub_vis.publish( marker );
		    	}
		    }

		else{
			continue;
    		}
    }
  // cout<<"Distance="<<minDistance<<"\n";
  // 	cout<<"Angle in Degree X axis="<<min_angle_radx*(180/3.14159265358979323846)<<"\n";
  // 	cout<<"Angle in Degree Y axis="<<min_angle_rady*(180/3.14159265358979323846)<<"\n";
  // 	cout<<"pointXcoordinate="<<xX<<"\n";
  // 	cout<<"pointYcoordinate="<<yY<<"\n";
  // 	cout<<"pointZcoordinate="<<zZ<<"\n";
  // 	cout<<"count="<<count<<"\n";
	
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input){

	filter_compute_distance_cloud(input);

	}

int main (int argc, char** argv){
	ros::init (argc, argv,"pub_mindistance");
	ros::NodeHandle nh;
	pub_vis= nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1,callback);
	ros::spin();
	}
