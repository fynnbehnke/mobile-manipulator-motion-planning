#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/TransformStamped.h>

class CameraControl
{
public:
    CameraControl(ros::NodeHandle nh) : nh_(nh), it_(nh)
    {
        init_params();

        // set up publishers and subscirbers
        state_pub = nh.advertise<std_msgs::Bool>("/butler/aruco_estimation/state", 1, true); // state publisher for control node
        cameraParam_sub = nh_.subscribe("/butler/ee_camera/camera_info", 1, &CameraControl::camParam_cb, this); // camera parameters from gazebo plugin
        image_sub_ = it_.subscribe("/butler/ee_camera/image_raw", 1, &CameraControl::cam_cb, this); // camera image from gazebo plugin
        detector_pub_ = it_.advertise("/butler/aruco_estimation/image", 1); // processed image publisher for visualization

        set_initial_pose();
    }

    ~CameraControl(){}

    // Initialize the default parameters
    void init_params(){
        // set values for ArUco tag reference points
        cv::Mat objPoints_temp(4, 1, CV_32FC3);
        objPoints_temp.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
        objPoints_temp.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
        objPoints_temp.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
        objPoints_temp.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
        objPoints = objPoints_temp.clone();


        // goal for mobile base
        base_goal.target_pose.header.frame_id = "map";
        base_goal.target_pose.pose.position.x = 1.3;
        base_goal.target_pose.pose.position.y = 0.0;
        tf2::Quaternion tempOrientation;
        tempOrientation.setRPY(0.0, 0.0, 0.0);
        base_goal.target_pose.pose.orientation.x = tempOrientation.getX();
        base_goal.target_pose.pose.orientation.y = tempOrientation.getY();
        base_goal.target_pose.pose.orientation.z = tempOrientation.getZ();
        base_goal.target_pose.pose.orientation.w = tempOrientation.getW();

        aruco_estimation_state.data = false;
    }

    void set_initial_pose(){
        // set up the MoveGroupInterface for MoveIt
        moveit::planning_interface::MoveGroupInterface move_group_interface("ur5");

        // control UR5 to initial pose
        std::vector<double> butler_scanPose = {0.0, -4*M_PI/6, M_PI/2, -M_PI/2, -M_PI/2, M_PI};
        move_group_interface.setJointValueTarget(butler_scanPose);
        move_group_interface.move();

        ROS_INFO("Manipulator target reached");

        // set up move base action client for base control
        goal_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);

        while(!goal_client->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        // control MIR100 to initial pose
        goal_client->sendGoal(base_goal);
    
        goal_client->waitForResult();

        if(goal_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Mobile base target reached");
            robot_ready = true;
        }else{
            ROS_WARN("MIR base failed to reach mobile base target");
        }
    }

    // Get camera parameters for solvePnP
    void camParam_cb(const sensor_msgs::CameraInfo::ConstPtr& cam_params){
        // camera matrix
        cv::Mat cameraTemp(3, 3, CV_64F);
        cameraTemp.at<double>(0,0) = cam_params->K.at(0);
        cameraTemp.at<double>(0,1) = cam_params->K.at(1);
        cameraTemp.at<double>(0,2) = cam_params->K.at(2);

        cameraTemp.at<double>(1,0) = cam_params->K.at(3);
        cameraTemp.at<double>(1,1) = cam_params->K.at(4);
        cameraTemp.at<double>(1,2) = cam_params->K.at(5);

        cameraTemp.at<double>(2,0) = cam_params->K.at(6);
        cameraTemp.at<double>(2,1) = cam_params->K.at(7);
        cameraTemp.at<double>(2,2) = cam_params->K.at(8);
        cameraMatrix = cameraTemp.clone();

        // distortion values
        cv::Mat distTemp(1, 5, CV_64F);
        distTemp.at<double>(0) = cam_params->D.at(0);
        distTemp.at<double>(1) = cam_params->D.at(1);
        distTemp.at<double>(2) = cam_params->D.at(2);
        distTemp.at<double>(3) = cam_params->D.at(3);
        distTemp.at<double>(4) = cam_params->D.at(4); 
        distCoeffs = distTemp.clone();

        params_ready = true;
        cameraParam_sub.shutdown(); // shutdown subscriber
        
        ROS_INFO("Got Params");
    }

    // Get camera image
    void cam_cb(const sensor_msgs::Image::ConstPtr& gazebo_frame){
            
        // transform sensor_msgs/Image to cv::Mat for OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(gazebo_frame, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // set current frame
        curr_frame = cv_ptr->image.clone();

        if(params_ready && robot_ready){
            // detect aruco-tag
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(curr_frame, dictionary, corners, ids, detectorParams);

            if (ids.size() > 0){
                cv::aruco::drawDetectedMarkers(curr_frame, corners, ids);

                // pose estimation
                int nMarkers = corners.size();
                std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);
                std::vector<cv::Mat> aruco_tf(nMarkers);
                
                // Calculate pose for each marker
                for (int i = 0; i < nMarkers; i++){
                    if(corners.at(i).size() < 4){
                        ROS_WARN("Not enough corners for SolvePnP");
                    }else{
                        // calculate tf to marker frame
                        solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                        

                        cv::Rodrigues(rvecs.at(i), aruco_tf.at(i));

                        tf2::Matrix3x3 tf_rot(  aruco_tf.at(i).at<double>(0, 0), aruco_tf.at(i).at<double>(0, 1), aruco_tf.at(i).at<double>(0, 2),
                                                aruco_tf.at(i).at<double>(1, 0), aruco_tf.at(i).at<double>(1, 1), aruco_tf.at(i).at<double>(1, 2),
                                                aruco_tf.at(i).at<double>(2, 0), aruco_tf.at(i).at<double>(2, 1), aruco_tf.at(i).at<double>(2, 2));
                        tf2::Vector3 tf_trans(  tvecs.at(i)[0], tvecs.at(i)[1], tvecs.at(i)[2]);
                        
                        // set up tf broadcaster between camera link and ArUco tag
                        broadcast_tf(tf_trans, tf_rot);

                        if(!aruco_estimation_state.data){
                            aruco_estimation_state.data = true;
                            state_pub.publish(aruco_estimation_state);

                            ROS_INFO_STREAM("Calculated TF to Aruco-Tag");
                        }
                    }
                }
                
                // Draw axis for each marker
                for(unsigned int i = 0; i < ids.size(); i++) {
                    cv::drawFrameAxes(curr_frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
                }
            }

        }

        cv_ptr->image = curr_frame.clone();

        detector_pub_.publish(cv_ptr->toImageMsg());
    }

    void broadcast_tf(tf2::Vector3 translation, tf2::Matrix3x3 rotation){
        static tf2_ros::TransformBroadcaster tf_broadcaster;
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "camera_link";
        transform.child_frame_id = "aruco_black";
        transform.transform.translation.x = translation[0];
        transform.transform.translation.y = translation[1];
        transform.transform.translation.z = translation[2];
        
        tf2::Vector3 rpy;
        rotation.getRPY(rpy[0], rpy[1], rpy[2]);
        tf2::Quaternion orientation;
        orientation.setRPY(rpy[0], rpy[1], rpy[2]);

        transform.transform.rotation.x = orientation.getX();
        transform.transform.rotation.y = orientation.getY();
        transform.transform.rotation.z = orientation.getZ();
        transform.transform.rotation.w = orientation.getW();
        
        tf_broadcaster.sendTransform(transform);
    }

private:
    ros::NodeHandle nh_;

    ros::Publisher state_pub;
    ros::Subscriber cameraParam_sub;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher detector_pub_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *goal_client;
    move_base_msgs::MoveBaseGoal base_goal;

    const cv::Ptr<cv::aruco::DetectorParameters> detectorParams = new cv::aruco::DetectorParameters();
    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Mat curr_frame;
    cv::Mat objPoints;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    float markerLength = 0.2;
    bool params_ready = false;
    bool robot_ready = false;

    std_msgs::Bool aruco_estimation_state;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Camera_Node");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(4);
    
    spinner.start();

    CameraControl cam_ctrl(nh);

    ros::waitForShutdown();

    return 0;   // copyright Fynn Behnke
}