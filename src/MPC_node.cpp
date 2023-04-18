#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <mpc_track/trajectory.h>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <mpc_track/mpc.h>
#include <mpc_track/control.h>
using namespace std;

double freq,L,V_DESIRED;//采样频率，车辆轴距，期望速度
int prediction_horizon;//预测时域
bool limit_v_and_kesi;//是否限幅(对于阿卡曼转向车辆需要限幅，全向车倒还好)
double initial_x,initial_y,initial_yaw,initial_v,initial_kesi;//初始化车辆位姿，速度和前轮转角
double slow_LEVE1_DISTANCE,slow_LEVE2_DISTANCE,slow_LEVE1_V,slow_LEVE2_V,goal_tolerance_DISTANCE;//定义二级减速距离和速度
#define pi acos(-1)
#define T 1/freq //采样时间 
 
vehicleState update_state(U control, vehicleState car) {
	car.v = control.v;
	car.kesi = control.kesi;
	car.x += car.v * cos(car.yaw) * T;
	car.y += car.v * sin(car.yaw) * T;
	car.yaw += car.v / L * tan(car.kesi) * T;
	return car;
}
 
class Path {
private:
	vector<waypoint> path;
public:
	//添加新的路径点
	void Add_new_point(waypoint& p)
	{
		path.push_back(p);
	}
 
	void Add_new_point(vector<waypoint>& p) 
	{
		path = p;
	}
 
	//路径点个数
	unsigned int Size()
	{
		return path.size();
	}
 
	//获取路径点
	waypoint Get_waypoint(int index)
	{
		waypoint p;
		p.ID = path[index].ID;
		p.x = path[index].x;
		p.y = path[index].y;
		p.yaw = path[index].yaw;
		return p;
	}

	vector<waypoint> Get_waypoints(){
		return path;
	}
 

	// 搜索路径点, 将小车到起始点的距离与小车到每一个点的距离对比，找出最近的目标点索引值
	int Find_target_index(vehicleState state)
	{
		double min = abs(sqrt(pow(state.x - path[0].x, 2) + pow(state.y - path[0].y, 2)));
		int index = 0;
		for (int i = 0; i < path.size(); i++)
		{
			double d = abs(sqrt(pow(state.x - path[i].x, 2) + pow(state.y - path[i].y, 2)));
			if (d < min)
			{
				min = d;
				index = i;
			}
		}
 
		//索引到终点前，当（机器人与下一个目标点的距离Lf）小于（当前目标点到下一个目标点距离L)时，索引下一个目标点
		if ((index + 1) < path.size())
		{
			double current_x = path[index].x; double current_y = path[index].y;
			double next_x = path[index + 1].x; double next_y = path[index + 1].y;
			double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
			double L_1 = abs(sqrt(pow(state.x - next_x, 2) + pow(state.y - next_y, 2)));
			//ROS_INFO("L is %f,Lf is %f",L,Lf);
			if (L_1 < L_)
			{
				index += 1;
			}
		}
		return index;
	}

	vector<waypoint> get_local_path(int index){
		vector<waypoint> local_path;
		int num = Size();
		//ROS_INFO("%d", num);
		for (int i = 0; i<prediction_horizon; i++){
			if(index + i<num){
				local_path.push_back(Get_waypoint(index + i));
			}
			else{
				local_path.push_back(Get_waypoint(num-1));
			}
			/*ROS_INFO("local_path hs %d numbers\n"
				"the ID is %d\nthe x is %f\n"
				"the y is %f\nthe yaw is %f\n",
				local_path.size(),local_path[i].ID,local_path[i].x,local_path[i].y,local_path[i].yaw);*/
		}
		return local_path;
	}
 
};
 
class MPC_node {
private:
	//car
	vehicleState car;//小车状态
	U control;//小车控制量[v,kesi]
	int lastIndex;//最后一个点索引值
	waypoint lastPoint;//最后一个点信息
	string action;//小车目前动作：跟踪或跟踪完成(tracking or reach goal!)

	//ROS
	ros::Subscriber path_sub;//订阅路径，消息类型为nav_msgs::Path
	ros::Publisher vel_pub;//发布速度信息，消息类型为geometry_msgs::Twist
	ros::Publisher actual_state_pub;//发布小车实际位姿，消息类型为geometry_msgs::Pose2D
	ros::Publisher visual_state_pub;//向rviz发布小车虚拟轨迹，消息类型为visualization_msgs::Marker
	ros::Publisher predictive_state_pub;//向rviz发布小车预测轨迹，消息类型为visualization_msgs::Marker
	ros::ServiceClient mpc_optimization; //建立优化客户端
	geometry_msgs::Point visual_state_pose;
	visualization_msgs::Marker visual_state_trajectory;
	nav_msgs::Path predictive_state_path;
	geometry_msgs::Pose2D actual_pose;
	geometry_msgs::Twist vel_msg;
	vector<geometry_msgs::Pose2D> state_prediction;
	int temp;//计数，达到终点时，用于判断控制器是否需要关闭

 
public:
    Path* path;

	MPC_node(ros::NodeHandle& nh)//初始化中添加轨迹、小车初始位姿
	{
        	path = new Path();
        
		//ROS:
		path_sub = nh.subscribe("path",10,&MPC_node::addpointcallback,this);
		vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
		visual_state_pub = nh.advertise<visualization_msgs::Marker>("visualization_pose",10);
		predictive_state_pub = nh.advertise<nav_msgs::Path>("predictive_pose",10);
		actual_state_pub = nh.advertise<geometry_msgs::Pose2D>("MPC_pose",10);
		mpc_optimization = nh.serviceClient<mpc_track::mpc>("mpc_optimization");
		//robot state initialize:
		car.x = initial_x;car.y = initial_y;car.yaw = initial_yaw;car.v = initial_v;car.kesi = initial_kesi;
		action = "the car is tracking!!";
	}
 
	~MPC_node() {
		delete(path);
	}
 
	void addpointcallback(const nav_msgs::Path::ConstPtr& msg){
		vector<waypoint> waypoints;
		for(int i=0;i<msg->poses.size();i++){
			waypoint waypoint;
			//ROS_INFO("THE PATH[%d]'s ID is %d",i,msg->poses[i].header.seq);
			waypoint.ID = msg->poses[i].header.seq;
			waypoint.x = msg->poses[i].pose.position.x;
			waypoint.y = msg->poses[i].pose.position.y;
			//获取角度
			double roll,pitch,yaw;
	    		tf::Quaternion quat;
	    		tf::quaternionMsgToTF(msg->poses[i].pose.orientation, quat);
	    		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			waypoint.yaw = yaw;
			waypoints.push_back(waypoint);
		}
		path->Add_new_point(waypoints);//路径点vector数组传到path类中
		lastIndex = path->Size() - 1;
		lastPoint = path->Get_waypoint(lastIndex);
	}
 
	double slow_judge(double distance) {
		if (distance>=slow_LEVE2_DISTANCE&&distance <= slow_LEVE1_DISTANCE) {
			return slow_LEVE1_V;
		}
		else if (distance>=goal_tolerance_DISTANCE&&distance < slow_LEVE2_DISTANCE) {
			return slow_LEVE2_V;
		}
		else if (distance < goal_tolerance_DISTANCE) {
			action = "the car has reached the goal!";
			return 0.0;
		}
		else
		{
			return V_DESIRED;
		}
	}

	//参考状态获取函数
	vector<geometry_msgs::Pose2D> get_state_reference(int index){
		vector<geometry_msgs::Pose2D> state_reference;
		vector<waypoint> local_path = path->get_local_path(index);
		for (int i = 0; i<local_path.size();i++){
			geometry_msgs::Pose2D point;
			point.x = local_path[i].x;
			point.y = local_path[i].y;
			point.theta = local_path[i].yaw;
			state_reference.push_back(point);
		}
		//ROS_INFO("state_reference length is %d",state_reference.size());
		return state_reference;
	}

	//参考控制量获取函数
	vector<mpc_track::control> get_control_reference(int index, double distance){
		vector<mpc_track::control> control_reference;
		vector<waypoint> local_path = path->get_local_path(index);
		double v_judge = slow_judge(distance);
		for (int i = 0; i<local_path.size();i++){
			mpc_track::control uu;
			uu.v = v_judge;
			//接着计算对应的前轮转角参考量
			double K = cal_K(path->Get_waypoints(),local_path[i].ID);//计算曲率
			uu.kesi = atan2(L * K, 1);
			control_reference.push_back(uu);
			//ROS_INFO("the v_desired is %f\nthe kesi_desired is %f\n",uu.v, uu.kesi);
		}
		//ROS_INFO("control_reference length is %d",control_reference.size());
		return control_reference;
	}

	//控制器流程
	/*
	跟踪功能实现步骤：
        1.获取路径点，在while(ros::ok())函数下面ros::spinOnce()阶段订阅回调函数addpointcallback时完成
        2.搜索路径点
        3.获取局部路径state_reference信息，构造对应的期望控制量序列control_reference
	在构造control_reference中有减速判断，确定了期望速度和机器人当前动作
        4.将当前位姿、局部路径、对应期望控制量序列传入优化器中，采用server/client的形式调用优化服务(mpc_optimizer.py)
        5.计算速度和前轮转角
        6.发布话题，其中有角速度的计算
        7.用运动学模型计算小车下一时刻位姿
        8.判断是否需要关闭控制器，如果到达终点就关闭
        9.loop_rate.sleep()，结束一次循环，准备开始下一次个跟踪工作
	*/

	void MPC_track() {
		//ROS_INFO("path size is: %d",path->Size());
		if(path->Size()!=0){
			//搜索路径点
			int target_index = path->Find_target_index(car);
			//ROS_INFO("target index is : %d", target_index);
	
			//获取路径点信息，构造期望控制量(分为参考状态和参考控制量)
			//参考状态
			vector<geometry_msgs::Pose2D> state_reference = get_state_reference(target_index);
			//参考控制量
			double v_distance = abs(sqrt(pow(car.x - lastPoint.x, 2) + pow(car.y - lastPoint.y, 2)));
			ROS_INFO("the distance is %f", v_distance);
			ROS_INFO("%s",action.c_str());//机器人动作
			vector<mpc_track::control> control_reference = get_control_reference(target_index, v_distance);

			//调用优化服务计算控制量
			mpc_optimize(state_reference, control_reference);
			if(slow_judge(v_distance)==0){control.v = 0;control.kesi = 0;}//判断，期望速度为0，则机器人停下
			ROS_INFO("the speed is: %f,the kesi is: %f", control.v, control.kesi);
			ROS_INFO("the angular speed is: %f", control.v*tan(control.kesi)/L);
			//ROS_INFO("the car position is x: %f, y: %f", car.x, car.y);
			
			Prediction_Path_set();//设置Path属性
			//话题发布
			PUB();
			
			//小车位姿状态更新
			car = update_state(control, car);

			//控制器关闭判断
			shutdown_controller();

			ROS_INFO("--------------------------------------");
		}
	}

	//调用优化服务
	void mpc_optimize(vector<geometry_msgs::Pose2D> state_reference, vector<mpc_track::control> control_reference){
		double solve_time;
		
		mpc_track::mpc srv;
		srv.request.state_ref = state_reference;
		srv.request.control_ref = control_reference;
		geometry_msgs::Pose2D position;
		position.x = car.x; position.y = car.y; position.theta = car.yaw;
		srv.request.state = position;
		if(mpc_optimization.call(srv)){
			control.v = srv.response.opt_control.v;
			control.kesi = srv.response.opt_control.kesi;
			state_prediction = srv.response.state_pre;
			solve_time = srv.response.solve_time;
			ROS_INFO("optimization solve time is %fs", solve_time);
		}
		else{
			ROS_ERROR("can't connect to optimization!");
			control.v = control.kesi = 0;
			state_prediction.clear();
			solve_time = 0;
		}
	}
 
	//控制启停函数
	void node_control() {
        	ros::Rate loop_rate(freq);
		Marker_set();//设置Marker属性
		//设置tf坐标转换
		tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Quaternion q;

		while (ros::ok()) {
			
			transform.setOrigin(tf::Vector3(car.x, car.y, 0));
			q.setRPY(0, 0, car.yaw);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "car"));

			ros::spinOnce();
			MPC_track();
			loop_rate.sleep();
		}
	}

	void PUB(){
		visual_state_pose.x = car.x; visual_state_pose.y = car.y;
		actual_pose.x = car.x; actual_pose.y = car.y; actual_pose.theta = car.yaw;
		vel_msg.linear.x = control.v; vel_msg.angular.z = control.v*tan(control.kesi)/L;//横摆角速度为w = v*tan(kesi)/L
		visual_state_trajectory.points.push_back(visual_state_pose);//visualization_msgs::Marker为一个容器，所以现需要向里面push_back结构体，再发布
		visual_state_pub.publish(visual_state_trajectory);//发布虚拟轨迹
		predictive_state_pub.publish(predictive_state_path);//发布预测轨迹
		vel_pub.publish(vel_msg);//发布速度
		actual_state_pub.publish(actual_pose);//发布位姿
	}

	void shutdown_controller(){
		if(action == "the car has reached the goal!"){
			temp+=1;
			if(temp >= 20){
				ROS_WARN("shutdown the MPC controller!");
				temp = 0;
				ros::shutdown();
			}
		}
	}

	void Marker_set(){
		//设置消息类型参数
		visual_state_trajectory.header.frame_id = "map";
		visual_state_trajectory.header.stamp = ros::Time::now();
		visual_state_trajectory.action = visualization_msgs::Marker::ADD;
		visual_state_trajectory.ns = "MPC";
		//设置点的属性
		visual_state_trajectory.id = 0;
		visual_state_trajectory.type = visualization_msgs::Marker::POINTS;
		visual_state_trajectory.scale.x = 0.02;
		visual_state_trajectory.scale.y = 0.02;
		visual_state_trajectory.color.r = 1.0;
		visual_state_trajectory.color.a = 1.0;
	}

	void Prediction_Path_set(){
		predictive_state_path.poses.clear();
		predictive_state_path.header.seq = 0;
		predictive_state_path.header.stamp = ros::Time::now();
		predictive_state_path.header.frame_id = "map";
		for(int i = 0;i<state_prediction.size();i++){
			geometry_msgs::PoseStamped predictive_state_pose;
			predictive_state_pose.header.seq = i + 1;
			predictive_state_pose.header.stamp = ros::Time::now();
			predictive_state_pose.header.frame_id = "map";
			predictive_state_pose.pose.position.x = state_prediction[i].x;
			predictive_state_pose.pose.position.y = state_prediction[i].y;
			predictive_state_pose.pose.orientation = tf::createQuaternionMsgFromYaw(state_prediction[i].theta);
			predictive_state_path.poses.push_back(predictive_state_pose);
		}
	}
};
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "MPC_node");
	ros::NodeHandle n;
	ros::NodeHandle n_prv("~");

	n_prv.param<double>("freq",freq,20);
	n_prv.param<double>("L",L,0.2);
	n_prv.param<double>("V_DESIRED",V_DESIRED,0.5);
	n_prv.param<double>("initial_x",initial_x,0.0);
	n_prv.param<double>("initial_y",initial_y,2.0);
	n_prv.param<double>("initial_yaw",initial_yaw,0.0);
	n_prv.param<double>("initial_v",initial_v,0.0);
	n_prv.param<double>("initial_kesi",initial_kesi,0.1);
	n_prv.param<double>("slow_LEVE1_DISTANCE",slow_LEVE1_DISTANCE,5.0);
	n_prv.param<double>("slow_LEVE2_DISTANCE",slow_LEVE2_DISTANCE,2.0);
	n_prv.param<double>("goal_tolerance_DISTANCE",goal_tolerance_DISTANCE,0.1);
	n_prv.param<double>("slow_LEVE1_V",slow_LEVE1_V,0.35);
	n_prv.param<double>("slow_LEVE2_V",slow_LEVE2_V,0.15);
	n_prv.param<bool>("limit_v_and_kesi",limit_v_and_kesi,false);
	n_prv.param<int>("prediction_horizon",prediction_horizon,13);

	MPC_node* node = new MPC_node(n);
	node->node_control();
	return (0);
}

 
