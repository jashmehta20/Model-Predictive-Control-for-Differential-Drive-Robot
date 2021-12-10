#include <ros/ros.h> 
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

void robot_odom(const nav_msgs::Odometry msg);

// variable declaration
ros::Publisher robot_vel_pub,errors_pub,desired_traj_pub; // node publishers
tf::Point Odom_pos;    //odometry position (x,y)
double Odom_yaw;    //odometry orientation (yaw)

geometry_msgs::Pose2D qd,robot_pose,err;
geometry_msgs::Twist vel_msg;

double vel_desired,angular_desired,vel_robot,angular_robot,u1,u2,saturation_sigma,t,ts;
double a,b,ep;
double kx,ky,kz;
double xd,yd,xdd,ydd;
// double freq=2*M_PI/30;
double freq=M_PI/60;
double max_vel_robot = 0.22; // TuR_barlebot3 maximum linear velocity
double max_angular_robot = 2.8;  // TuR_barlebot3 maximum angular velocity



int main(int argc, char **argv)
{

	// ROS NODE INITIALISATION
    ros::init(argc, argv, "MPC_Node");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("/odom", 1000 , robot_odom);
    robot_vel_pub = n.advertise <geometry_msgs::Twist> ("/cmd_vel",1000);
	errors_pub = n.advertise <geometry_msgs::Pose2D> ("/errors_pub",1000);
	desired_traj_pub = n.advertise <geometry_msgs::Pose2D> ("/desired_traj_pub",1000);

	ros::Rate loop_rate(10); // 10Hz

	// All subscribers have to start
	while (errors_pub.getNumSubscribers() == 0 || desired_traj_pub.getNumSubscribers() == 0 
		|| robot_vel_pub.getNumSubscribers() == 0 )
	{
		loop_rate.sleep();
	}

	t = 0.0;
	ts = 0.1;				// Sampling time for mpc controller

	VectorXd u_ref(2),uf(2),e(3),k_mpc(2),c(2),Q(3),R(2);
	
	MatrixXd Ac(3,3),		// Continuous state space model
			A(3,3),  		// Discrete Time state space model 
			Ar(3,3),		// Reference error dynamics
			B(3,2),			// Discrete Time state space model
			Z(3,2),			// Zeros Matrix
			Q_bar(12,12),	
			R_bar(8,8),
			F(12,3),		// Robot tracking prediction error state space representation
			G(12,8),		// Horizon Matrix -> Robot tracking prediction error state space representation
			Fr(12,3),		// Reference tracking error state space representation 
			K1(8,8),
			k2(8,3),
			k_final(2,3),
			iden(3,3);
			int qr;
			float va;
			// qr=3;		// 4
			// va=0.5;
			qr=3;		// 4
			va=0.49;


	while(t<=130 && ros::ok())
	{
		
		// calculate the desired trajectory coordinates and velocities
		// Oval Path 

		// qd.x = -1.0 + cos(freq*t);
		// qd.y = 0.0 + sin(freq*t);
		// xd = -freq*sin(freq*t); yd = freq*cos(freq*t);
		// xdd = -freq*freq*cos(freq*t);ydd = -freq*freq*sin(freq*t);
		// qd.theta = atan2(yd,xd);
		// vel_desired = (sqrt(pow(xd,2) + pow(yd,2)));
		// angular_desired = ( (xd * ydd) - (yd * xdd) )/ (pow(xd,2) + pow(yd,2));
		// u_ref << vel_desired,angular_desired;

		// Lisajous path
		qd.x = 1.1 + 2*sin(freq*t);
		qd.y = 0.9 + 0.7*sin(2*freq*t);
		xd = freq*2*cos(freq*t); 
		yd = 2*freq*0.7*cos(2*freq*t);
		xdd = -freq*freq*2*sin(freq*t);
		ydd = 4*-freq*freq*0.7*sin(2*freq*t);
		qd.theta = atan2(yd,xd);
		vel_desired = (sqrt(pow(xd,2) + pow(yd,2)));
		angular_desired = ( (xd * ydd) - (yd * xdd) )/ (pow(xd,2) + pow(yd,2));
		u_ref << vel_desired,angular_desired;

		desired_traj_pub.publish(qd); // publish the desired trajectory coordinates to the plotter node
		

		// calculate Tracking errors between the robot and desired trajectory coordinates
		err.x = (qd.x-robot_pose.x) * cos(robot_pose.theta) + (qd.y-robot_pose.y) * sin(robot_pose.theta);
		err.y = -(qd.x-robot_pose.x) * sin(robot_pose.theta) + (qd.y-robot_pose.y) * cos(robot_pose.theta);
		err.theta = qd.theta - robot_pose.theta;
		err.theta = atan2(sin(err.theta),cos(err.theta));// Wrap theta in -Pi to Pi 
		errors_pub.publish(err); // publish errors to the plotter node

		Z = MatrixXd::Zero(3,2);
		uf << vel_desired*cos(err.theta), angular_desired;

		e << err.x , err.y , err.theta;

		Ac << 0,			ts*u_ref(1),		0,
			-ts*u_ref(1),	0,			ts*u_ref(0), 
			0,				0,				0;

		iden << 1,0,0,
				0,1,0,
				0,0,1;

		A << Ac+iden; 	

		Ar << va,0,0,
			  0,va,0,
			  0,0,va;	

		B << ts,0,
			0,0,
			0,ts;

		Q << qr,qr,qr;	
		R << pow(10,-7),pow(10,-7);	

		Q_bar = Q.replicate(4,1).asDiagonal();
		R_bar = R.replicate(4,1).asDiagonal();

		// G << A*B,	Z,	Z,	Z,
		// 	 A*A*B,	A*B,	Z,	Z, 
		// 	 A*A*A*B,	A*A*B,	A*B,	Z,
		// 	 A*A*A*A*B,	A*A*A*B,	A*A*B,	A*B;		


		G << B,	Z,	Z,	Z,
			 A*B,	B,	Z,	Z, 
			 A*A*B,	A*B,	B,	Z,
			 A*A*A*B,	A*A*B,	A*B,	B;	

		// F << A*A,	A*A*A,	A*A*A*A,	A*A*A*A*A;
		F << A,	A*A,	A*A*A,	A*A*A*A;

		Fr << Ar,	Ar*Ar,	Ar*Ar*Ar,	Ar*Ar*Ar*Ar;

		K1 = G.transpose() * Q_bar * G + R_bar  ;
		k2 = K1.inverse()*(G.transpose() * Q_bar *(Fr-F));
		k_final << k2.row(0),k2.row(1);
		k_mpc = -k_final*e;

		c = k_mpc + uf;
		vel_robot = c(0);
		angular_robot = c(1);


		// A saturation function of the command velocities that preserves the  curvature
		// ref -> Tracking-error model-based predictive control for mobile robots
		// in real time Robotics and Autonomous Systems
		saturation_sigma = std::max((fabs(vel_robot)/max_vel_robot),std::max((fabs(angular_robot)/max_angular_robot),1.0));

		if (saturation_sigma == (fabs(vel_robot)/max_vel_robot))
		{	
			u1 = std::copysignf(1.0,vel_robot)*max_vel_robot;
			u2 = angular_robot/saturation_sigma;			
		}

		if (saturation_sigma == (fabs(angular_robot)/max_angular_robot))
		{		
			u2 = std::copysignf(1.0,angular_robot)*max_angular_robot;
			u1 = vel_robot/saturation_sigma;
		}

		if (saturation_sigma == 1)
		{			
			u1 = vel_robot;
			u2 = angular_robot;
		}			
		
	
		vel_msg.linear.x = u1;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =u2;
		robot_vel_pub.publish(vel_msg);
		t = t+0.1;

		ros::spinOnce();
		loop_rate.sleep();		
	}

		vel_msg.linear.x = 0;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =0;
		robot_vel_pub.publish(vel_msg);
    return 0;
}

// function to get robot position and orientation from the odom topic
void robot_odom(const nav_msgs::Odometry msg)
{
    tf::pointMsgToTF(msg.pose.pose.position, Odom_pos);
    Odom_yaw = tf::getYaw(msg.pose.pose.orientation);
	robot_pose.x = msg.pose.pose.position.x;
	robot_pose.y = msg.pose.pose.position.y;

    robot_pose.theta = Odom_yaw;   
}

