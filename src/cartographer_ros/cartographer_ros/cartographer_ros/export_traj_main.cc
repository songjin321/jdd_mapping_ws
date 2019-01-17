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

void ExportPbstream(const std::string& pbstream_filename) {
  carto::mapping::proto::PoseGraph pose_graph_proto = carto::io::DeserializePoseGraphFromFile(pbstream_filename);
  for (size_t trajectory_id = 0; trajectory_id < pose_graph_proto.trajectory().size();++trajectory_id) {
    const carto::mapping::proto::Trajectory& trajectory_proto = pose_graph_proto.trajectory(trajectory_id);
    Eigen::Matrix4d trans_map_base_0;
    for (int i = 0; i < trajectory_proto.node_size(); ++i) {
      const ::cartographer::mapping::proto::Trajectory_Node& node = trajectory_proto.node(i);
      const ::cartographer::transform::proto::Vector3d& translation=node.pose().translation();
      const ::cartographer::transform::proto::Quaterniond& rotation=node.pose().rotation();
      if (i == 0)
      {
         Eigen::Quaterniond quaternion_map_base_0(rotation.w(),rotation.x(),rotation.y(),rotation.z());
         Eigen::Translation3d translation_map_base_0(translation.x(), translation.y(), translation.z());
         trans_map_base_0 = (translation_map_base_0 * quaternion_map_base_0).matrix();
      }
      Eigen::Quaterniond quaternion_map_base(rotation.w(),rotation.x(),rotation.y(),rotation.z());
      Eigen::Translation3d translation_map_base(translation.x(), translation.y(), translation.z());
      Eigen::Matrix4d trans_map_base = (translation_map_base * quaternion_map_base).matrix();
      trans_map_base = trans_map_base_0.inverse() * trans_map_base;
      // std::cout << std::setprecision(12) << "trans_map_base translation = \n" << trans_map_base.block<3, 1>(0, 3) << std::endl 
      // << "trans_map_base euler = \n" << trans_map_base.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;

      // trans_base_output
      Eigen::AngleAxisd rotation_base_output(0.0, Eigen::Vector3d::UnitZ());
      Eigen::Translation3d translation_base_output(-0.825123, 0.000, -0.942);
      Eigen::Matrix4d trans_base_output = (translation_base_output * rotation_base_output).matrix();

      // map_utm (output frame)
      Eigen::AngleAxisd rotation_utm_output(-146.7835234/180.0*M_PI, Eigen::Vector3d::UnitZ());
      Eigen::Translation3d translation_utm_output(457074.7411138645, 4404764.041069881, 22.0);
      Eigen::Matrix4d trans_utm_map = (translation_utm_output * rotation_utm_output).matrix(); 
      // std::cout << std::setprecision(12) << "trans_utm_map translation = \n" << trans_utm_map.block<3, 1>(0, 3) << std::endl 
      // << "trans_utm_map euler = \n" << trans_utm_map.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;

      Eigen::Matrix4d trans_utm_output = trans_utm_map * trans_base_output.inverse()* trans_map_base * trans_base_output;
      Eigen::Quaterniond quaternion_utm_output(trans_utm_output.block<3, 3>(0, 0));

      ros::Time ros_time=::cartographer_ros::ToRos(cartographer::common::FromUniversal(node.timestamp()));
      double time=ros_time.sec+ros_time.nsec*1e-9;
      std::cout<< setprecision(16) 
      <<time 
      <<" "<<trans_utm_output.block<3, 1>(0, 3)[0]
      <<" "<<trans_utm_output.block<3, 1>(0, 3)[1]
      <<" "<<22.0
      <<" "<<quaternion_utm_output.x()
      <<" "<<quaternion_utm_output.y()
      <<" "<<quaternion_utm_output.z()
      <<" "<<quaternion_utm_output.w()<<std::endl;
    }

  }
}
// convert a pbstream to a tum file 
// input: output->base, map->utm
// output: output->utm
int main(int argc, char** argv) {
  if (argc != 2)
  {
    std::cout << "please specific pbstream file path!" << std::endl;
    return -1;
  }
  ExportPbstream(std::string(argv[1]));
  return 0;
}
