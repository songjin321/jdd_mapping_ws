#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros/split_string.h"
#include "geometry_msgs/TransformStamped.h"
#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "ros/ros.h"
#include <iostream>
#include <stdlib.h>
#include <iomanip>
using namespace std;

namespace carto = ::cartographer;
static void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny_cosp, cosy_cosp);

  yaw =yaw*180.0/M_PI ; 
  pitch = pitch*180.0/M_PI; 
  roll = roll*180.0/M_PI;
}

  void ExportPbstream(const std::string& pbstream_filename) {
    carto::io::ProtoStreamReader reader(pbstream_filename);
    carto::io::ProtoStreamDeserializer deserializer(&reader);

    carto::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
    
    for (size_t trajectory_id = 0; trajectory_id < pose_graph_proto.trajectory().size();
        ++trajectory_id) {
      const carto::mapping::proto::Trajectory& trajectory_proto =
          pose_graph_proto.trajectory(trajectory_id);

      for (int i = 0; i < trajectory_proto.node_size(); ++i) {
        const ::cartographer::mapping::proto::Trajectory_Node& node = trajectory_proto.node(i);
      // node.pose() contains the pose...
        const ::cartographer::transform::proto::Vector3d& translation=node.pose().translation();
        const ::cartographer::transform::proto::Quaterniond& rotation=node.pose().rotation();
        Eigen::Quaterniond quaternion(rotation.w(),rotation.x(),rotation.y(),rotation.z());
        double roll=0,pitch=0,yaw=0;
        toEulerAngle(quaternion,roll,pitch,yaw);
        // Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(0,1,2);
        // double yaw = euler[2]*180.0/M_PI ; 
        // double pitch = euler[1]*180.0/M_PI; 
        // double roll = euler[0]*180.0/M_PI;
        ros::Time ros_time=::cartographer_ros::ToRos(cartographer::common::FromUniversal(node.timestamp()));
        double time=ros_time.sec+ros_time.nsec/(double)1e9;
        //std::cout.setf(std::ios::fixed,std::ios::floatfield);
        //std::cout<<  <<","<<translation.x()<<","<<translation.y()<<","<<translation.z()<<","<<roll<<","<<pitch<<","<<yaw<<","<<std::endl;
        std::cout<< setprecision(16) <<time <<" "<<translation.x()<<" "<<translation.y()<<" "<<translation.z()<<" "<<rotation.x()<<" "<<rotation.y()<<" "<<rotation.z()<<" "<<rotation.w()<<std::endl;
        //std::cout<<quaternion.w()<<","<<quaternion.x()<<","<<quaternion.y()<<","<<quaternion.z()<<","<<rotation.w()<<","<<rotation.x()<<","<<rotation.y()<<","<<rotation.z()<<std::endl;
      }

    }
  }
  int main(int argc, char** argv) {
    ExportPbstream("/home/johnnysun/.ros/finish1.pbstream");
    return 0;
  }
