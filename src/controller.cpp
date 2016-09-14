#include "controller.h"
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <boost/math/special_functions/sign.hpp>

controller::controller()
{
}

void
controller::cmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cmd_pose = msg;
}

void
controller::vrepPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  vrep_pose = msg;
}

void
controller::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  if (msg->poses.size())
      m_path = msg;
  else
      m_path = NULL;
}

void 
controller::absposeCallback(const apriltags_ros_sim::AprilTagDetectionArray::ConstPtr& msg)
{
    if(msg->detections.size())
    {   
        abs_pose = boost::make_shared<const apriltags_ros_sim::AprilTagDetection>(msg->detections[0]);
    }
    else
    {
        abs_pose = NULL;
    }
}

bool 
controller::init(ros::NodeHandle& nh)
{
// intialisation of the controller

// set up path subscriber, in callback function, subscribe the path topic which is a series of Poses
    m_pathSub = nh.subscribe<nav_msgs::Path>("path", 1, boost::bind(&controller::pathCallback, this, _1));
// set up pose subscriber, in callback function, subscribe current location of the robot. This pose estimation is perfectly right which can be used as control referrence as well as ground truth
    m_vrepPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("/vrep/pose", 1, boost::bind(&controller::vrepPoseCallback, this, _1));
// set up pose estimated by vision algorithm which should be periodically updated when barcode is detected
    m_absposeSub = nh.subscribe<apriltags_ros_sim::AprilTagDetectionArray>("tag_detections", 1, boost::bind(&controller::absposeCallback, this, _1));
// set up pose advertiser, publish the estimated pose, the estimation is done by odometry and vision algorithm
    m_posePub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
// set up motor command publisher, which publish a point. first two element are used as command for left motor and right motor respectively
    m_cmdPub = nh.advertise<geometry_msgs::Point>("command", 1);
// set up robot status publisher, which tell robot's current status to the upper computer. In this case, the status is reach or not reach waypoint
    m_statusPub = nh.advertise<std_msgs::Bool>("status", 1);

// initialize start pose, first element is x displayment, second element is y displayment, and third one is current heading direction in rad
    m_state  << 0,
                3,
                0.0;

// initialize motor command to be zero
    cmd.x=0;
    cmd.y=0;
    cmd.z=0;

// initialize waypoint reach status
    block_path_receiver = false;
// initialize robot status
    m_move = IDLE;
// forward or backward status are both set to be false
    fix_forward = false;
    fix_turn = false;
// set the barcode map resolution
    m_resolution = 1;
    return true;
}

bool 
controller::indicator(double theta_x, double theta_y, double thres)
{
    return (2*3.1415926-fabs(theta_x-theta_y) < thres)||(fabs(theta_x - theta_y) < thres);
}

double 
controller::angle(double goal, double start)
{
    if(fabs(goal-start) < 3.1415926)
       return goal-start;
    else if ((goal-start)>3.1415926)
       return goal-start - 2*3.1415926;
    else
       return goal-start + 2*3.1415926;
}

Eigen::Matrix3f
controller::qToRotation(Eigen::Quaternion<float> q)
{
    Eigen::Matrix3f temp;

    temp << 1-2*(q.y()*q.y()+q.z()*q.z()), 2*(q.x()*q.y()-q.w()*q.z())  , 2*(q.w()*q.y()+q.x()*q.z())  ,
            2*(q.x()*q.y()+q.w()*q.z())  , 1-2*(q.x()*q.x()+q.z()*q.z()), 2*(q.y()*q.z()-q.w()*q.x())  ,
            2*(q.x()*q.z()-q.w()*q.y())  , 2*(q.y()*q.z()+q.w()*q.x())  , 1-2*(q.x()*q.x()+q.y()*q.y());
    return temp;
}

double
controller::getYawFromMat(Eigen::Matrix3f mat)
{
    return atan2(mat(1,0), mat(0,0));
}

double 
controller::mod2pi(double angle)
{
    if(fabs(angle) < 3.1415926)
         return angle;
    else if (angle < -3.1415926)
         return 2*3.1415926+angle;
    else
         return angle-2*3.1415926;
}

void
controller::running(void)
{
    cycle_start = ros::Time::now();

    if (m_path && !block_path_receiver)
    {
//  new path received, refine raw path message to extracted path which can be executed by robot one by one 
        ROS_INFO("Path Received!");
        nav_msgs::Path temp_path = *m_path;
        std::vector<geometry_msgs::PoseStamped> temp_path_vector;
        extracted_path.clear();
        for( std::vector<geometry_msgs::PoseStamped>::iterator itr = temp_path.poses.begin() ; itr != temp_path.poses.end(); ++itr)
        {
            temp_path_vector.push_back(*itr);
        }
        while(!temp_path_vector.empty())
        {
            extracted_path.push_back(temp_path_vector.back());
            temp_path_vector.pop_back();
        }
//  the robot status block_path_receiver received, which means the robot can not receive the new path message until current mission is completed.
        block_path_receiver = true;
        m_statusPub.publish(block_path_receiver);
    }

    switch (m_move)
    {
        case FORWARD:
        {
        // compute the linear vel of the robot
            v =  sqrt(dist) * 0.75/(1+0.2*k*k) *10000/3.1415926;
        // compute the angular vel of the robot w = (vr-vl)/B, B means baseline length
            gain = 0.3265*v*k;
        // restrict the angular vel to be less than a set value
            if (fabs(gain) > 800)
            {
                gain = boost::math::sign(gain) * 800;
            }
        // restrict the linear vel to be less than a set value
            if(fabs(v)>1000)
            {
                v = boost::math::sign(v)*1000;
            }
        // set the final vr and vl and publish it
            cmd.x = v+gain;
            cmd.y = v-gain;
            m_cmdPub.publish(cmd);
        // jump out forward status if the euler distance between current position and waypoint position is less than 30mm
            if (sqrt((m_state[0]- m_cmd[0])*(m_state[0]-m_cmd[0]) + (m_state[1] -m_cmd[1]) * (m_state[1]-m_cmd[1])) < 0.04&&indicator(m_state[2],m_cmd[2], 0.03))
                m_move = IDLE;

            break;
        }

        case BACKWARD:
        {
        // compute the linear vel of the robot
            v =  sqrt(dist) * 0.75/(1+0.2*k_back*k_back) *10000/3.1415926;
        // compute the angular vel of the robot w = (vr-vl)/B, B means baseline length
            gain = 0.3265*v*k_back;
        // restrict the angular vel to be less than a set value
            if (fabs(gain) > 800)
            {
                gain = boost::math::sign(gain) * 800;
            }
        // restrict the linear vel to be less than a set value
            if(fabs(v)>1000)
            {
                v = boost::math::sign(v)*1000;
            }
        // set the final vr and vl and publish it
            cmd.x = -v+gain;
            cmd.y = -v-gain;
            m_cmdPub.publish(cmd);
        // jump out backward status if the euler distance between current position and waypoint position is less than 30mm
            if (sqrt((m_state[0]- m_cmd[0])*(m_state[0]-m_cmd[0]) + (m_state[1] -m_cmd[1]) * (m_state[1]-m_cmd[1])) < 0.04&&indicator(m_state[2],m_cmd[2], 0.03))
                m_move = IDLE;
            break;

        }

        case TURN:
        {
        // compute the linear vel of the robot and restrict the linear vel to be less than a set value
            v = cos(alpha) * dist* p3 *10000/3.1415926;
            if(fabs(v)>300)
            {
                v = boost::math::sign(v)*300;
            }
        // compute the angular vel of the robot w = (vr-vl)/B, B means baseline length and restrict the angular vel to be less than a set value
            gain = 3265*(p3*angle(m_cmd[2],m_state[2]))/3.1415926;
            if (fabs(gain) > 400)
            {
                gain = boost::math::sign(gain) * 400;
            }
        // set the final vr and vl and publish it
            cmd.x = v+gain;
            cmd.y = v-gain;
            m_cmdPub.publish(cmd);
        // jump out the turn status if the angle between current heading direction and waypoint heading direction is less than 1 degree
            if (indicator(m_state[2],m_cmd[2], 0.01))
               m_move = IDLE;
            break;


        }

        case LIFTFORK:
        {
        // jump in lift and fork status, executed lift and fork comission
            cmd.x = 0;
            cmd.y = 0;
            printf("LIFT and FORK!\n");
            sleep(10);
            m_move = IDLE;

            break;
        }

        case IDLE:
        {   
        // robot in idle status
        // check current path to follow is empty or not, if it is empty, publish block_path_receive status to be false and allow robot to receive new path, if it is not, execute next commanded waypoint in the path to follow
           if (extracted_path.size()!=0)
           {
               geometry_msgs::PoseStamped temp_pose = extracted_path.back();
               float yaw_ = 2*atan2(temp_pose.pose.orientation.z,temp_pose.pose.orientation.w);
               m_cmd << temp_pose.pose.position.x,
                        temp_pose.pose.position.y,
                        angle(yaw_,0);

               printf("Next Commanded Pose is (%f, %f, %f)\n", m_cmd[0], m_cmd[1], m_cmd[2]);
               // check next command waypoint is in the front of the robot or behind the robot, or the robot need to pure rotate to reach the next commanded waypoint, or lift and fork
               if ( (fabs(m_cmd[0] - m_state[0])>0.5) || (fabs(m_cmd[1] - m_state[1])>0.5) )
               {
                   if (fabs(m_cmd[0] - m_state[0])>0.5)
                   {
                       if (cos(m_state[2]) *  (m_cmd[0] - m_state[0]) > 0)
                           m_move = FORWARD;
                       else
                           m_move = BACKWARD;
                   }
                   else
                   {
                       if (sin(m_state[2]) *  (m_cmd[1] - m_state[1]) > 0)
                           m_move = FORWARD;
                       else
                           m_move = BACKWARD;

                   }
                   if (m_move == FORWARD)
                       printf("Move Forward!\n");
                   else
                       printf("Move Backward!\n");
                }
                else if (fabs(m_cmd[2] - m_state[2])>0.5)
                {
                    m_move = TURN;
                    printf("Turn Around!\n");
                }
                else if (temp_pose.pose.position.z!=0)
                    m_move = LIFTFORK;
                else
                    m_move = IDLE;
               // pop back the extracted_path vector
               extracted_path.pop_back();
           }
           else
           {
               // wait for new path and publish current pose
               block_path_receiver = false;
               geometry_msgs::PoseStamped pose_msg;
         
               pose_msg.header.frame_id = "world";
               pose_msg.header.stamp = ros::Time::now();
               pose_msg.pose.position.x = m_state[0];
               pose_msg.pose.position.y = m_state[1];
               if (block_path_receiver)
                  pose_msg.pose.position.z = 1;
               else
                  pose_msg.pose.position.z = 0;

               pose_msg.pose.orientation.w = cos(m_state[2]/2);
               pose_msg.pose.orientation.x = 0;
               pose_msg.pose.orientation.y = 0;
               pose_msg.pose.orientation.z = sin(m_state[2]/2);
               m_posePub.publish(pose_msg);
               printf("IDLE!\n");
               sleep(1);
               m_move = IDLE;
           }

           seq++;

            cmd.x = 0;
            cmd.y = 0;
            m_cmdPub.publish(cmd);
            break;
        }
    }

    vel.first = cmd.x;
    vel.second = cmd.y;
    usleep(30000);

    cycle_period = (ros::Time::now() - cycle_start).toSec();
    // odometry update(encoder read)
    m_state[0] = m_state[0] + (vel.second+vel.first)/2 * cos(m_state[2]) * 0.018849555 * cycle_period/60;
    m_state[1] = m_state[1] + (vel.second+vel.first)/2 * sin(m_state[2]) * 0.018849555 * cycle_period/60;        
    m_state[2] = m_state[2] + (vel.first - vel.second)/0.653 * 0.018849555 * cycle_period/60;


    if (abs_pose)
    {
    // if barcode is detected, update absolute current pose
        id = abs_pose->id;
        if (id != id_old)
        id_old = id;
        Eigen::Quaternion<float> quat;
        quat.w() = abs_pose->pose.pose.orientation.w,
        quat.x() = abs_pose->pose.pose.orientation.x,
        quat.y() = abs_pose->pose.pose.orientation.y,
        quat.z() = abs_pose->pose.pose.orientation.z;

        Eigen::Matrix3f Rotation;
        Eigen::Vector3f translation;
        translation << -abs_pose->pose.pose.position.x,
                       -abs_pose->pose.pose.position.y,
                       abs_pose->pose.pose.position.z;

        Rotation = qToRotation(quat);

        translation = -Rotation.inverse()*translation;

    // transform matrix between camera frame and robot base frame
        Eigen::Matrix3f FixTF;
        FixTF << 1, 0,  0,
                 0, -1, 0,
                 0, 0, -1;

        Rotation = Rotation.inverse()*FixTF;

        geometry_msgs::Point diff_msgs;
        diff_msgs.x = translation[0] + m_resolution * (id%10) - m_state[0];
        diff_msgs.y = translation[1] + m_resolution * floor(id/10.0) - m_state[1];
        diff_msgs.z = getYawFromMat(Rotation) - m_state[2];


        m_state[0] = translation[0] + m_resolution * (id%10) -4.5;
        m_state[1] = translation[1] + m_resolution * floor(id/10.0) -4.5;
        m_state[2] = getYawFromMat(Rotation);
    }

    else if(vrep_pose)
    {
    // get vrep estimated pose which is absolutely right
        Eigen::Quaternion<float> quat_1;
        quat_1.w() = vrep_pose->pose.orientation.w,
        quat_1.x() = vrep_pose->pose.orientation.x,
        quat_1.y() = vrep_pose->pose.orientation.y,
        quat_1.z() = vrep_pose->pose.orientation.z;

        Eigen::Matrix3f Rotation_1;
        Eigen::Vector3f translation_1;
        translation_1 << vrep_pose->pose.position.x,
                       vrep_pose->pose.position.y,
                       vrep_pose->pose.position.z;

        Rotation_1 = qToRotation(quat_1);
        m_state[0] = translation_1[0];
        m_state[1] = translation_1[1];
        m_state[2] = getYawFromMat(Rotation_1);
    }

    // publish current estimated pose by the robot
    geometry_msgs::PoseStamped pose_msg;
         
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = m_state[0];
    pose_msg.pose.position.y = m_state[1];
    if (block_path_receiver)
        pose_msg.pose.position.z = 1;
    else
        pose_msg.pose.position.z = 0;

    pose_msg.pose.orientation.w = cos(m_state[2]/2);
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = sin(m_state[2]/2);

    m_posePub.publish(pose_msg);

    // prepare the feedback data for differential control
    delta_y = m_cmd[1]-m_state[1];
    if(fabs(m_cmd[1]-m_state[1]) > 1.6)
    {
        delta_y = boost::math::sign(m_cmd[1]-m_state[1]) * 1.6;
    }

    delta_x = m_cmd[0]-m_state[0];
    if(fabs(m_cmd[0]-m_state[0]) > 1.6)
    {
        delta_x = boost::math::sign(m_cmd[0]-m_state[0]) * 1.6;
    }

    beta = angle(m_cmd[2], atan2(delta_y, delta_x));

    alpha = angle(m_state[2], atan2(delta_y, delta_x));

    beta1 = angle(m_cmd[2]+3.1415926, atan2(delta_y, delta_x));

    alpha1 = angle(m_state[2]+3.1415926, atan2(delta_y, delta_x));


    dist = sqrt(delta_x*delta_x + delta_y*delta_y);

    k = -1/dist * (k2*(alpha-atan2(-k1*beta,1)) + sin(alpha)*(1 + k1/(1+(k1*beta) * (k1*beta))));

    k_back = -1/dist * (k2*(alpha1-atan2(-k1*beta1,1)) + sin(alpha1)*(1 + k1/(1+(k1*beta1) * (k1*beta1))));
}

void 
controller::shutdown()
{


}

void 
controller::close(void)
{
}
