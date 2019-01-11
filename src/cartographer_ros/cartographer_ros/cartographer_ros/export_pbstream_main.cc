#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/transform.h"
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
#include <vector>
#include <fstream>

using namespace std;

namespace carto = ::cartographer;
struct Path_Pose{
    double time;
    double x;
    double y;
    double z;
    double yaw; //radian
};
// generate a pbstream file using given path file and a exsiting pbstream file
int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cout << "please specific a pbstream file, a path file and a result file path!" << std::endl;
        return -1;
    }
    std::string pbstream_filename(argv[1]);
    std::string path_filename(argv[2]);
    std::string result_filename(argv[3]);
    // read path file to a map
    std::vector<Path_Pose> poses;
    std::ifstream pathfile(path_filename);
    double pose_time, t_x, t_y, t_z, q_x, q_y, q_z, q_w;
    while (pathfile >> pose_time >> t_x >> t_y >> t_z >> q_x >> q_y >> q_z >> q_w)
    {
        Eigen::Quaterniond q(q_w, q_x, q_y, q_z);
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        Path_Pose pose;
        pose.time = pose_time;
        pose.x = t_x;
        pose.y = t_y;
        pose.z = t_z;
        pose.yaw = euler[2];
        // std::cout << "yaw = " << pose.yaw << std::endl;
        poses.push_back(pose);
    }

    //  construct a pbstream
    carto::io::ProtoStreamReader reader(pbstream_filename);
    carto::io::ProtoStreamDeserializer deserializer(&reader);
    carto::mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
    carto::mapping::proto::AllTrajectoryBuilderOptions options_proto = deserializer.all_trajectory_builder_options();
    for (size_t trajectory_id = 0; trajectory_id < pose_graph_proto.trajectory().size(); ++trajectory_id)
    {
        carto::mapping::proto::Trajectory *trajectory_proto = pose_graph_proto.mutable_trajectory(trajectory_id);
        for (int i = 0; i < trajectory_proto->node_size(); ++i)
        {
            Eigen::Quaterniond quaternion_map_base_0;
            ::cartographer::mapping::proto::Trajectory_Node* node = trajectory_proto->mutable_node(i);
            ::cartographer::transform::proto::Vector3d* translation = node->mutable_pose()->mutable_translation();
            ::cartographer::transform::proto::Quaterniond* rotation = node->mutable_pose()->mutable_rotation();
            ros::Time ros_time = ::cartographer_ros::ToRos(cartographer::common::FromUniversal(node->timestamp()));
            double time = ros_time.sec + ros_time.nsec * 1e-9;

            // set the translation and rotation of node based time and path file
            double true_x_utm_ouput=0, true_y_utm_ouput=0, true_z_utm_ouput=0, true_yaw_utm_ouput=0;
 
            for (int i = 0; i < poses.size(); i++)
            {
                if (time < poses[i].time)
                {
                    double t1 = poses[i-1].time;
                    double t2 = poses[i].time;
                    true_x_utm_ouput = poses[i-1].x + (poses[i].x - poses[i-1].x)/(t2-t1)*(time-t1);
                    true_y_utm_ouput = poses[i-1].y + (poses[i].y - poses[i-1].y)/(t2-t1)*(time-t1);
                    true_z_utm_ouput = poses[i-1].z + (poses[i].z - poses[i-1].z)/(t2-t1)*(time-t1);
                    true_yaw_utm_ouput = poses[i-1].yaw + (poses[i].yaw - poses[i-1].yaw)/(t2-t1)*(time-t1);
                    break;
                }
            }

            // initial output->utm
            Eigen::AngleAxisd rotation_utm_output_init(poses[0].yaw, Eigen::Vector3d::UnitZ());
            Eigen::Translation3d translation_utm_output_init(poses[0].x, poses[0].y, poses[0].z);
            Eigen::Matrix4d trans_utm_output_init = (translation_utm_output_init * rotation_utm_output_init).matrix(); 

            Eigen::AngleAxisd rotation_utm_output(true_yaw_utm_ouput, Eigen::Vector3d::UnitZ());
            Eigen::Translation3d translation_utm_output(true_x_utm_ouput, true_y_utm_ouput, true_z_utm_ouput);
            Eigen::Matrix4d trans_utm_output = (translation_utm_output * rotation_utm_output).matrix(); 
            //std::cout << std::setprecision(12) << "trans_utm_output translation = \n" << trans_utm_output.block<3, 1>(0, 3) << std::endl 
            //<< "trans_utm_output euler = \n" << trans_utm_output.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;

            Eigen::Matrix4d trans_map_base = trans_utm_output_init.inverse() * trans_utm_output;
            //std::cout << std::setprecision(12) << "trans_map_base translation = \n" << trans_map_base.block<3, 1>(0, 3) << std::endl 
            //<< "trans_map_base euler = \n" << trans_map_base.block<3, 3>(0, 0).eulerAngles(0, 1, 2) << std::endl;
            Eigen::Quaterniond quat_map_base(trans_map_base.block<3, 3>(0, 0));
            translation->set_x(trans_map_base.block<3, 1>(0, 3)[0]);
            translation->set_y(trans_map_base.block<3, 1>(0, 3)[1]);
            translation->set_z(trans_map_base.block<3, 1>(0, 3)[2]);
            rotation->set_x(quat_map_base.x());
            rotation->set_y(quat_map_base.y());
            rotation->set_z(quat_map_base.z());
            rotation->set_w(quat_map_base.w());
        }
    }

    // write constructed pose graph to file
    carto::io::ProtoStreamWriter writer(result_filename);
    carto::mapping::proto::SerializationHeader header;
    header.set_format_version(1);
    writer.WriteProto(header);
    carto::mapping::proto::SerializedData proto;
    *proto.mutable_pose_graph() = pose_graph_proto;
    writer.WriteProto(proto);
    *proto.mutable_all_trajectory_builder_options() = options_proto;
    writer.WriteProto(proto);
    return 0;
}