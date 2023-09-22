#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>


class ButlerController
{

public:
    ButlerController(ros::NodeHandle nh) : nh_(nh)
    {
        init_params();

        move_group_interface_ = new moveit::planning_interface::MoveGroupInterface("ur5");

        tfListener = new tf2_ros::TransformListener(buff);

        tcp_link_ = "tcp";
        base_link_ = "ur_base_link";

        move_group_interface_->setPoseReferenceFrame("camera_link");
        currPose_ = move_group_interface_->getCurrentPose();

        state_sub_ = nh_.subscribe("/butler/aruco_estimation/state", 1, &ButlerController::estimation_state_cb, this);
    }

    ~ButlerController()
    {
        delete move_group_interface_;
    }

    void init_params(){
        grippingPose_.header.frame_id = "morobot_gripping_point";
        grippingPose_.header.stamp = ros::Time::now();

        grippingPose_.pose.position.x = 0.0;
        grippingPose_.pose.position.y = 0.0;
        grippingPose_.pose.position.z = 0.0;

        tf2::Quaternion tempOrientation;
        tempOrientation.setRPY(0.0, 0.0, 0.0);
        grippingPose_.pose.orientation.x = tempOrientation.getX();
        grippingPose_.pose.orientation.y = tempOrientation.getY();
        grippingPose_.pose.orientation.z = tempOrientation.getZ();
        grippingPose_.pose.orientation.w = tempOrientation.getW();
    }

    // control robot if ArUco tag estimation is successfull
    void estimation_state_cb(const std_msgs::Bool::ConstPtr& state_msg){
        estimation_state = state_msg->data;
        ROS_INFO_STREAM("Estimated Aruco-Tag pose!");

        buff.setUsingDedicatedThread(true);

        if(estimation_state){
            // check if tf between camera and gripping point is broadcasted
            try{
                buff.canTransform("camera_link", "morobot_gripping_point", ros::Time::now(), ros::Duration(1.0));
            }catch(tf2::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }

            ROS_INFO_STREAM("TF received!");

            // compute gripping pose in UR5 base link frame
            geometry_msgs::PoseStamped desired_pose; 
            try{
                buff.canTransform("ur_base_link", "morobot_gripping_point", ros::Time::now(), ros::Duration(1.0));
                desired_pose = buff.transform(grippingPose_, "ur_base_link");
            }catch(tf2::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }

            printPose(desired_pose);

            KDL::Rotation goal_rot = KDL::Rotation::Quaternion(desired_pose.pose.orientation.x, desired_pose.pose.orientation.y, desired_pose.pose.orientation.z, desired_pose.pose.orientation.w);
            KDL::Vector goal_trans(desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z);

            // get the tree of the butler_robot urdf file. Might have to edit path
            KDL::Tree ur5_tree;
            if (!kdl_parser::treeFromFile("/home/fynnb/catkin_ws/src/mobile-manipulator-motion-planning/ur_butler_moveit/config/gazebo_butler_robot.urdf", ur5_tree)){            
                    ROS_ERROR("Failed to construct kdl tree");
                    exit(-1);
            }

            // Generate a kinematic chain from the robot base to its tcp
            KDL::Chain ur5_chain;
            ur5_tree.getChain("ur_base_link", "camera_link", ur5_chain);

            // set up inverse kinematics solver
            KDL::ChainFkSolverPos_recursive fk_solver(ur5_chain);
            KDL::ChainIkSolverVel_pinv vel_ik_solver(ur5_chain, 0.0001, 1000);
            KDL::ChainIkSolverPos_NR ik_solver(ur5_chain, fk_solver, vel_ik_solver, 1000);

            KDL::Frame goal_pose(goal_rot, goal_trans);

            KDL::JntArray jnt_init_state(6);
            jnt_init_state(0) = 0.0;
            jnt_init_state(1) = 0.0;
            jnt_init_state(2) = 0.0;
            jnt_init_state(3) = 0.0;
            jnt_init_state(4) = 0.0;
            jnt_init_state(5) = 0.0;

            // Compute inverse kinematics, since chomp and stomp calculate in joint-space
            KDL::JntArray jnt_pos_goal(6);
            ik_solver.CartToJnt(jnt_init_state, goal_pose, jnt_pos_goal);

            std::vector<double> goalConfig(6);
            toCorrectAngle(jnt_pos_goal(0));
            goalConfig[0] = jnt_pos_goal(0);
            toCorrectAngle(jnt_pos_goal(1));
            goalConfig[1] = jnt_pos_goal(1);
            toCorrectAngle(jnt_pos_goal(2));
            goalConfig[2] = jnt_pos_goal(2);
            toCorrectAngle(jnt_pos_goal(3));
            goalConfig[3] = jnt_pos_goal(3);
            toCorrectAngle(jnt_pos_goal(4));
            goalConfig[4] = jnt_pos_goal(4);
            toCorrectAngle(jnt_pos_goal(5));
            goalConfig[5] = jnt_pos_goal(5);
            
            // Move UR5 to targetPose
            moveTarget(goalConfig);
        }
    }

    void toCorrectAngle(double &angle){
        // limit angle to [-pi; pi]
        if(angle > M_PI)
            do{
                angle -= 2 * M_PI;
            }while(angle > M_PI);
        else if(angle < -M_PI)
            do{
                angle += 2 * M_PI; 
            }while(angle < -M_PI);
    }

    void moveTarget(std::vector<double> targetConfig)
    {   
        // print target configuration and move robot
        for(int i = 0; i < targetConfig.size(); ++i){
            std::cout << "J" << i << ": " << targetConfig[i] << std::endl;
        }

        move_group_interface_->setJointValueTarget(targetConfig);
        move_group_interface_->move();
    }

    void printPose(geometry_msgs::PoseStamped pose)
    {

        std::cout << "\tPosition" << std::endl;
        std::cout << "\tx: " << pose.pose.position.x << std::endl;
        std::cout << "\ty: " << pose.pose.position.y << std::endl;
        std::cout << "\tz: " << pose.pose.position.z << std::endl;
        std::cout << "\tOrientation" << std::endl;
        std::cout << "\tx: " << pose.pose.orientation.x << std::endl;
        std::cout << "\ty: " << pose.pose.orientation.y << std::endl;
        std::cout << "\tz: " << pose.pose.orientation.z << std::endl;
        std::cout << "\tw: " << pose.pose.orientation.w << std::endl;
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;

    moveit::planning_interface::MoveGroupInterface *move_group_interface_;

    std::string tcp_link_;
    std::string base_link_;

    tf2_ros::Buffer buff;
    tf2_ros::TransformListener* tfListener;

    geometry_msgs::PoseStamped currPose_;
    geometry_msgs::PoseStamped grippingPose_;
    geometry_msgs::PoseStamped targetPose_;

    std::vector<geometry_msgs::PoseStamped> trajectory_;

    bool estimation_state = false;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Control_Node");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(4);
    
    spinner.start();

    ButlerController robot_ctrl(nh);

    ros::waitForShutdown();

    return 0;   // copyright Fynn Behnke
}