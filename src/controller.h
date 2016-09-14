#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include "apriltags_ros_sim/AprilTagDetectionArray.h"
#include "math.h"
#include <Eigen/Dense>
#include "boost/shared_ptr.hpp"
#include "boost/make_shared.hpp"


class controller
{
  public:
    controller(void);
    bool init(ros::NodeHandle& nh);
    void running(void);

    void close(void);
    void shutdown(void);

    enum MOVE
    {
        FORWARD = 0,
        TURN = 1,
        IDLE = 2,
        LIFTFORK = 3,
        BACKWARD = 4,
    };


  private:
    void cmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void absposeCallback(const apriltags_ros_sim::AprilTagDetectionArray::ConstPtr& msg);
    void vrepPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    ros::Subscriber m_cmdSub;
    ros::Subscriber m_pathSub;
    ros::Subscriber m_absposeSub;
    ros::Subscriber m_vrepPoseSub;

    apriltags_ros_sim::AprilTagDetectionArray tag_detections;
    apriltags_ros_sim::AprilTagDetection::ConstPtr abs_pose;

    MOVE m_move;
    Eigen::Vector3d m_state;
    Eigen::Vector3d m_cmd;

    geometry_msgs::PoseStamped m_lastwaypoint;
  
    geometry_msgs::PoseStamped m_waypoint;

    std::vector<geometry_msgs::PoseStamped> extracted_path;
  

    ros::Publisher m_posePub;
    ros::Publisher m_cmdPub;
    ros::Publisher m_diffPub;
    ros::Publisher m_statusPub;

    geometry_msgs::PoseStamped::ConstPtr m_pose;
    geometry_msgs::PoseStamped::ConstPtr vrep_pose;

    geometry_msgs::PoseStamped::ConstPtr cmd_pose;
    nav_msgs::Path::ConstPtr m_path;

    geometry_msgs::Point cmd;

    double gain = 0;
    double dist = 0;
    double alpha = 0; 
    double beta = 0;
    double alpha1 = 0; 
    double beta1 = 0;

    double v;

    double p3 = 0.5;
    double p4 = 1.6;
    double p5 = -0.3;
    double delta_y = 0.0;
    double delta_x = 0.0;

    double k1 = 1;
    double k2 = 3;
    double gamma = 0.1;
    double lambda = 2;
    double k;
    double k_back;
    int id=-1;
    int id_old=-1;

    bool block_path_receiver;

    std::pair <int,int> vel;
    ros::Time cycle_start;
    double cycle_period;
    double m_resolution;
    bool indicator(double theta_x, double theta_y, double thres);
    double angle(double goal, double start);
    Eigen::Matrix3f qToRotation(Eigen::Quaternion<float> q);
    double getYawFromMat(Eigen::Matrix3f mat);
    bool fix_forward, fix_turn;
    int seq = 0;
    double mod2pi(double angle);
};
