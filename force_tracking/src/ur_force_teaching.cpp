#include "../include/force_leader_follower/ur_force_teaching.hpp"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <list>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include <random>



#include <fstream>
#include <iostream>
#include <queue>
#include <std_msgs/Bool.h>


namespace teaching {


    URForceTeaching::URForceTeaching(ros::NodeHandle _nh, const std::string &_urdf_param, std::string &_prefix) {
        //init
        nh_ = _nh;
        prefix_ = _prefix;

        joint_size_ = 6;
        joint_names_.push_back("shoulder_pan_joint");
        joint_names_.push_back("shoulder_lift_joint");
        joint_names_.push_back("elbow_joint");
        joint_names_.push_back("wrist_1_joint");
        joint_names_.push_back("wrist_2_joint");
        joint_names_.push_back("wrist_3_joint");

        current_JntArr_.resize(joint_size_);
        for (int i = 0; i < joint_size_; ++i) {
            current_JntArr_(i) = 0;
        }


        //server pub sub
        pub_wrench_in_base_ = nh_.advertise<geometry_msgs::Wrench>("force_in_base", 1);
        pub_contact_force_ = nh_.advertise<std_msgs::Float64>("Contact_force", 1);
        sub_end_position_ = nh_.advertise<geometry_msgs::Point>("end_point", 1);
        pub_ref_force_ = nh_.advertise<std_msgs::Float64>("Ref_force", 1);

        sub_joint_state_ = nh_.subscribe(prefix_ + "joint_states", 1, &URForceTeaching::subJointStateCB, this);
//            sub_joint_voltage_ = nh_.subscribe(prefix_ + "joint_voltage",1, &URForceTeaching::subJointVoltageCB, this);
        sub_wrench_ = nh_.subscribe("ethdaq_data", 1, &URForceTeaching::subWrenchCB, this);
//        sub_io_state_ = nh_.subscribe("ur_driver/io_states", 1, &URForceTeaching::sub_io_state, this);


//            admittance_control1.initialize();
//            contact_dragging1.initialize();
//            velocity_pid_loop1.initialize();

        nh_.getParam("force_PID_loop", force_PID_data);
        kp = force_PID_data[0];
        ki = force_PID_data[1];
        kd = force_PID_data[2];
        limit_dx = 20;
        ROS_INFO_STREAM("force PID loop: "<<"kp="<<kp<<" ki="<<ki<<" kd="<<kd);        // 金属PI 参数

        write_data = false;
        name_suffix = 1;

        bur_sub_ = false;
        bwrench_sub_ = false;
        io_button_ = false;
        force_tracking_state_ = false;
        l1 = 0;

        //robot model
        urdf::Model robot_model;

        std::string full_urdf_xml;
        nh_.searchParam(_urdf_param, full_urdf_xml);
        std::string xml_string;
        if (nh_.getParam(full_urdf_xml, xml_string)) {
            nh_.param(full_urdf_xml, xml_string, std::string());
            robot_model.initString(xml_string);

            //limit
            min_jnt_.resize(joint_size_);
            max_jnt_.resize(joint_size_);
            vel_limit_.resize(joint_size_);
            //
            boost::shared_ptr<const urdf::Joint> joint;
            for (int i = 0; i < joint_size_; ++i) {
                joint = robot_model.getJoint(prefix_ + joint_names_[i]);
                min_jnt_(i) = joint->limits->lower;
                max_jnt_(i) = joint->limits->upper;
                vel_limit_(i) = joint->limits->velocity;
                ROS_INFO_STREAM("vel_limit " << vel_limit_(i));
            }
        }

        p_fk_solver_ = NULL;
        p_jac_solver_ = NULL;
        p_tracik_solver_ = NULL;
        KDL::Tree tree;  //
        if (kdl_parser::treeFromUrdfModel(robot_model, tree)) {
            std::string base_link = prefix_ + "base_link";
            std::string tip_link = prefix_ + "tool1";//todo
            if (tree.getChain(base_link, tip_link, chain_)) {
                std::vector<KDL::Segment> segments = chain_.segments;
                for (int x = 0; x < segments.size(); ++x) {
                    KDL::Segment seg = segments.at(x);
                    std::string name = seg.getName();
                    KDL::Joint jnt = seg.getJoint();

                    KDL::Vector axis = jnt.JointAxis();
                    KDL::Frame frame = seg.getFrameToTip();
                    double r, p, y;
                    frame.M.GetRPY(r, p, y);
                    //output
                    ROS_INFO_STREAM("KDL:: name=" << name << "; "
                                                  << "axis=[" << axis[0] << "," << axis[1] << "," << axis[2] << "]; "
                                                  << "frame.p=[" << frame.p[0] << "," << frame.p[1] << "," << frame.p[2]
                                                  << "]; "
                                                  << "frame.rpy=[" << r << "," << p << "," << y << "]; "
                    );
                }
                p_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
                p_jac_solver_ = new KDL::ChainJntToJacSolver(chain_);
            } else {
                ROS_FATAL("Couldn't find chain %s to %s", base_link.c_str(), tip_link.c_str());
            }
            //ik
            double timeout;
            nh_.param("trac_ik_timeout", timeout, 0.005);
            double eps = 1e-5;
            p_tracik_solver_ = new TRAC_IK::TRAC_IK(base_link, tip_link, _urdf_param, timeout, eps);
        } else {
            ROS_FATAL("Failed to extract kdl tree from xml robot description");
        }
        //action client
//            client_ = new Client(prefix_+"follow_joint_trajectory", true);
//            client_speedj_ = new Client(prefix_+"pos_based_pos_traj_controller/follow_joint_trajectory", true);
        client_servoj_ = new Client(prefix_ + "pos_based_pos_traj_controller/follow_joint_trajectory", true);
        ROS_INFO("wait for server");
//            client_->waitForServer(ros::Duration());
        client_servoj_->waitForServer(ros::Duration());
        ROS_INFO("server connect! ");

        teaching_state_ = STOP;//初始化
        bstate_change_ = false;


        nh_.getParam("ef_twist_factor", ef_joint_factor_);
        nh_.getParam("plane_angle", plane_angle);
        // row and column of the Y_data
//            nh_.getParam("row_Y",row_Y);
//            nh_.getParam("col_Y",col_Y);
        row_Y = 784;
        col_Y = 15;

//            ROS_INFO_STREAM("kp ki kd "<<kp<<" "<<ki<<" "<<kd);
        ROS_INFO_STREAM("plane_angle " << plane_angle);
        nh_.getParam("name_suffix", name_suffix);

        gravity_ = KDL::Vector(0, 0, -2.8);
        rot_FT_EE_ = KDL::Rotation::RPY(0,-M_PI/2, M_PI);//(0, 0, 1, 0, -1, 0, 1, 0, 0);//todo (0,-1,0,1,0,0,0,0,1);   //绕z轴旋转-90度得到FT坐标系


        ROS_INFO_STREAM(
                "frame_wrist3_base_ : \n" << rot_FT_EE_.data[0] << " " << rot_FT_EE_.data[1] << rot_FT_EE_.data[2]
                                          << "\n"
                                          << rot_FT_EE_.data[3] << " " << rot_FT_EE_.data[4] << rot_FT_EE_.data[5]
                                          << "\n"
                                          << rot_FT_EE_.data[6] << " " << rot_FT_EE_.data[7] << rot_FT_EE_.data[8]
                                          << "\n");
//            frame_EE_wrist3_.p = KDL::Vector(0, -0.163, 0);                // y方向平移0.0819mm ee_link to wrist3
//            frame_EE_wrist3_.p = KDL::Vector(0, -0.17762, 0);                //最长的 y方向平移175.62mm 末端长度
        frame_EE_wrist3_.p = KDL::Vector(0, -0.184, 0);                 //middle+ spring 160.5mm  160.5+3 = 163.5
//            frame_EE_wrist3_.p = KDL::Vector(0, 0.203, 0);                // y方向平移 0.203 0.0819+0.12+error

        frame_EE_wrist3_.M = KDL::Rotation::RPY(0, 0,M_PI / 2);      // 绕z轴旋转-90°
        frame_base_world_.p = KDL::Vector(0, 0, 0);
        std::vector<double> wrpy;
        nh_.getParam("base_world_RPY", wrpy);
        frame_base_world_.M = KDL::Rotation::RPY(0, 0,0); //KDL::Rotation::RPY(0,0,M_PI/4) * KDL::Rotation::RPY(0,M_PI/2,0) * KDL::Rotation::RPY(0,0,-M_PI/2);
//            KDL::Rotation test0 =  KDL::Rotation::RPY(M_PI, 0, M_PI);
//            std::cout<<"test0: ";
//            for (int j = 0; j <9 ; ++j) {
//                std::cout<<test0.data[j]<<" ";
//            }

        nh_.getParam("xyz_vel_factor", xyz_delta_velocity_);
        nh_.getParam("rot_vel_factor", rot_delta_velocity_);
//            xyz_delta_velocity_ = 0.008;//todo  0.005
//            rot_delta_velocity_ = 0.35;//todo  0.35
        delta_time_ = 0.008;//todo 0.001

        current_cart_vel_.resize(6);
        current_cart_vel_.setZero();
        vel_err_.resize(6);
        vel_err_.setZero();
        translation_acc_ = 0.015;// 0.015;
        rotation_acc_ = 0.05;//0.05;

        repair_wrench_.force = KDL::Vector(-0.1, -1.0, -12.9);
        repair_wrench_.torque = KDL::Vector(-0.073, -0.067, 0.021);
        end_rpy_ = KDL::Vector(0, 0, 0);
        end_point_ = KDL::Vector(0, 0, 0);

        //EF
        nh_.getParam("ef_wrench_factor", ef_wrench_factor_);
//            ef_wrench_factor_ = 0.2;
        bfirst_ef_wrench_ = true;
        nh_.getParam("ef_twist_factor", ef_twist_factor_);
//            ef_twist_factor_ = 0.1;
        bfirst_ef_twist_ = true;


    }

    bool URForceTeaching::init() {
        return true;
    }

    void URForceTeaching::run() {
        ros::spin();
    }








    void URForceTeaching::load_data_file(int start_row, double time_rate) {
        int row_Y = 3137;
        int col_Y = 10;

        double **a;
        a = new double *[row_Y];
        for (int i = 0; i < row_Y; i++)
            a[i] = new double[col_Y];

        std::ifstream fin("/home/gx/Polishing/src/force_tracking/Y_data_GP.txt");
        for (int i = 0; i < row_Y; i++) {
            for (int j = 0; j < col_Y; j++) {
                fin >> a[i][j];
                Y[i][j] = a[i][j];
//                     ROS_INFO_STREAM("Y"<<"["<<i<<"]"<<"["<<j<<"] = "<<Y[i][j]);

            }
        }
        ROS_INFO_STREAM("start time: " << Y[start_row][9] << "  Row of Y" << row_Y);
        //[x y z x1 y1 z1 force std vel time]   x1 y1 z1 为法向量
        fin.close();
        double temp1 = Y[start_row][9];
        for (int i = 0; i < row_Y; ++i) {
            Y[i][9] = Y[i][9] - temp1;   // time column to zero from start_row;
            ROS_INFO_STREAM("Y time: " << i << " " << Y[i][9]);
            Y[i][9] = Y[i][9] * time_rate;
            Y[i][8] = Y[i][8] / 1000 / time_rate;
            Y[i][0] +=0.2326-0.1947 +0.007;  //offset
            Y[i][1] -= 0.3378-0.2806;
        }
    }

    void URForceTeaching::moveto_first_trajectory_point(int start_row) {

        KDL::Frame first_traj_point, p0, p1;

        first_traj_point.p.data[0] = Y[start_row][0];
        first_traj_point.p.data[1] = Y[start_row][1];
        first_traj_point.p.data[2] = Y[start_row][2];
//            for(int i=0;i<9;++i ) {
//                first_traj_point.M.data[i] = Y[start_row][i+3];
//                ROS_INFO_STREAM("M_"<<i<<" "<<first_traj_point.M.data[i]);
//            }
        KDL::Vector xyz1 = {Y[start_row][3], Y[start_row][4], Y[start_row][5]};
        transform_Vector2R(xyz1, first_traj_point.M);

        double offset0 = 0.05, offset1 =  0.001;
        p0 = first_traj_point;
        p1 = first_traj_point;

        p0.p.data[0] = p0.p.data[0] - offset0 * p0.M.data[1];
        p0.p.data[1] = p0.p.data[1] - offset0 * p0.M.data[4];
        p0.p.data[2] = p0.p.data[2] - offset0 * p0.M.data[7];
        p1.p.data[0] = p1.p.data[0] - offset1 * p1.M.data[1];
        p1.p.data[1] = p1.p.data[1] - offset1 * p1.M.data[4];
        p1.p.data[2] = p1.p.data[2] - offset1 * p1.M.data[7];
        KDL::JntArray j_first, j_p0, j_p1;
        j_first.resize(6);
        j_p0.resize(6);
        j_p1.resize(6);


        double first_jnt_d[6] = {-130, -70.01, -129.70, -49.90, 145.08, -57.64};
        for (int i = 0; i < 6; ++i) {
            j_first(i) = first_jnt_d[i] * M_PI / 180;
        }
        servoj_moveto(j_first, 3, true);    //移动到固定角度点
        ros::Duration(0.3).sleep();
        bur_sub_ = false;
        do {
            ros::spinOnce();
        } while (bur_sub_ == false);
        p_tracik_solver_->CartToJnt(current_JntArr_, p0, j_p0);   //ikine 可能出错
        servoj_moveto(j_p0, 2, true);
        ros::Duration(0.3).sleep();

        bur_sub_ = false;
        do {
            ros::spinOnce();
        } while (bur_sub_ == false);
        p_tracik_solver_->CartToJnt(current_JntArr_, p1, j_p1);   //ikine
        servoj_moveto(j_p1, 2, true);

//        bur_sub_ = false;
//        do{
//            ros::spinOnce();
//        }while(bur_sub_ == false);
//        p_tracik_solver_->CartToJnt(current_JntArr_, first_traj_point, joints);   //ikine
//        moveTo(joints,3,false);
    }

    void URForceTeaching::interpolate_traj_test(int start_row, int row_Y, double time_now, KDL::JntArray &ref_joints) {
        int ss = 0;
        for (int i = start_row; i < row_Y - 1; ++i) {
            if (time_now > Y[i][9] & time_now <= Y[i + 1][9]) {
                ss = i;
                break;
            }
        }
//        ROS_INFO_STREAM("current row: " << ss);
        //ikine for Y[i] and Y[i+1]   其实可以先离线把所有点的逆解求出来，节约时间 todo 二维数据如何传递地址？ 动态二维数组创建，列数一定，行数可变
        KDL::Frame p1_Y, p2_Y,pt;
        KDL::JntArray p1_joints, p2_joints;
        for (int i = 0; i < 3; ++i) {
            p1_Y.p.data[i] = Y[ss][i];
            p2_Y.p.data[i] = Y[ss + 1][i];
        }
//            for (int i = 0;i<9;++i){
//                p1_Y.M.data[i] = Y[ss][i+3];
//                p2_Y.M.data[i] = Y[ss+1][i+3];
//            }
        KDL::Vector xyz1 = {Y[ss][3], Y[ss][4], Y[ss][5]};
        KDL::Vector xyz2 = {Y[ss + 1][3], Y[ss + 1][4], Y[ss + 1][5]};
        transform_Vector2R(xyz1, p1_Y.M);
        transform_Vector2R(xyz2, p2_Y.M);

        p_tracik_solver_->CartToJnt(current_JntArr_, p1_Y, p1_joints);
        p_tracik_solver_->CartToJnt(current_JntArr_, p2_Y, p2_joints);
        // 插值 t1 - p1     t2 - p2      time_now - ref_joints
        double t1 = Y[ss][9];
        double t2 = Y[ss + 1][9];
        double delta_t0 = t2 - t1;
        double k_time_joints = 0;


//        for (int i = 0; i < 6; ++i) {
//            k_time_joints = (p2_joints(i) - p1_joints(i)) / delta_t0;
//            ref_joints(i) = k_time_joints * (time_now - t1) + p1_joints(i);
////                ROS_INFO_STREAM("t1:"<<t1<<" "<<p1_joints(i)<<" time now:"<<time_now<<" Ref_j "<<ref_joints(i)
////                                     <<" t2:"<<t2<<" "<<p2_joints(i));
//        }
// cart interpolate
        KDL::Vector xyz_a = {0,0,0};  //normal
        KDL::Vector XYZ_1 = {Y[ss][0],Y[ss][1],Y[ss][2]};
        KDL::Vector XYZ_2 = {Y[ss+1][0],Y[ss+1][1],Y[ss+1][2]};
        double k = 0;
        for (int j = 0; j < 3; ++j) {
            k = (xyz2[j] -xyz1[j])/delta_t0;
            xyz_a[j] = k*(time_now - t1)+xyz1[j];
            k = (XYZ_2[j] -XYZ_1[j])/delta_t0;
            pt.p.data[j] = k*(time_now - t1)+XYZ_1[j];
        }
        double f1 = Y[ss][6],f2 =Y[ss+1][6];
        k = (f2-f1)/(t2-t1);
        ref_force_ = k*(time_now-t1)+f1;
        double std1 = Y[ss][7],std2 =Y[ss+1][7];
        k = (std2-std1)/(t2-t1);
        std_f_ = k*(time_now-t1)+std1;

        transform_Vector2R(xyz_a, pt.M);



        double contact_force = - current_wrench_.force.data[2];
        double delta_f2x = 0;
        contact_adjust.step(ref_force_, contact_force, delta_f2x);   // 力的误差经PID，转换为法向位移
        delta_f2x = delta_f2x/1000;
//        delta_f2x = delta_f2x-0.001;// 手动补偿
        sum_delta_f2x_ = delta_f2x;
//        sum_delta_f2x_ = 0;
        KDL::Vector delta_position_z_ee = KDL::Vector(0, 0, sum_delta_f2x_);
        ROS_INFO_STREAM("delta_f2x = "<<sum_delta_f2x_<<"    ref_F = "<<ref_force_<<"    cur_F = "<<contact_force);
        KDL::Vector position_EE = rot_FT_EE_ * delta_position_z_ee;
        KDL::Frame position_frame_EE_base = frame_wrist3_base_ * frame_EE_wrist3_;
        KDL::Vector delta_position_z_base = position_frame_EE_base.M * position_EE;
        pt.p = {pt.p.data[0]+delta_position_z_base[0],pt.p.data[1]+delta_position_z_base[1],pt.p.data[2]+delta_position_z_base[2]};

        p_tracik_solver_->CartToJnt(current_JntArr_, pt, ref_joints);

//            ROS_INFO_STREAM("current time " << time_now<<" calc-x"<< pt.p.data[0]<<"real-x"<<end_point_.data[0]);





        if (ss > row_Y - 500)
            end_trajectory_state = true;
//
//
//
//            int i = 0;
//            for(int s = 0;s<784-1;++s){
//                if((point.x>=Y[s][0] & point.x<=Y[s+1][0])|(point.x<=Y[s][0] & point.x>=Y[s+1][0]) ) //假设x为row_Y单调函数
//                    break;
//                i = s;
//            }
//            double kf = (Y[i+1][12]- Y[i][12])/(Y[i+1][0]-Y[i][0]);   // F
//            double ks = (Y[i+1][13]- Y[i][13])/(Y[i+1][0]-Y[i][0]);   // std of F
//            double kv = (Y[i+1][14]- Y[i][14])/(Y[i+1][0]-Y[i][0]);   // velocity
//            double v1;
//            F_r   = kf*(point.x - Y[i][0])+Y[i][12];
//            std_F = ks*(point.x - Y[i][0])+Y[i][13];
//            v1    = kv*(point.x - Y[i][0])+Y[i][14];
//            double tan_v[3] = {point.x - Y[i][0],point.y - Y[i][1],point.z - Y[i][2]};
//            double norm_tan_v = sqrt(tan_v[0]*tan_v[0]+tan_v[1]*tan_v[1]+tan_v[2]*tan_v[2]);
//            V_r[0] = tan_v[0]/norm_tan_v*v1;
//            V_r[1] = tan_v[1]/norm_tan_v*v1;
//            V_r[2] = tan_v[2]/norm_tan_v*v1;
//            ROS_INFO_STREAM("F  : "<<Y[i][12]<<" "<<Y[i+1][12]<<" interpolate value "<<F_r);
//            ROS_INFO_STREAM("std: "<<Y[i][13]<<" "<<Y[i+1][13]<<" interpolate value "<<std_F);
//            ROS_INFO_STREAM("v  : "<<Y[i][14]<<" "<<Y[i+1][14]<<" interpolate value "<<v1);
//            double distance_1 = sqrt((Y[i+1][0]-Y[i][0])*(Y[i+1][0]-Y[i][0])+(Y[i+1][1]-Y[i][1])*(Y[i+1][1]-Y[i][1])+(Y[i+1][2]-Y[i][2])*(Y[i+1][2]-Y[i][2]));
//            double time_1 =distance_1/(Y[i+1][14]+Y[i][14]) * 2;  //计算两个点之间所用时间（假设为直线，匀加速运动）
//            delta_sita[0] = Y[i+1][15]/time_1;
//            delta_sita[1] = Y[i+1][16]/time_1;
//            delta_sita[2] = Y[i+1][17]/time_1;
//






    }




//    void URForceTeaching::sub_io_state(ur_msgs::IOStates io_state) {
//        bool io_button = io_state.digital_in_states.at(16).state;
//        int io_button_num = io_state.digital_in_states.at(16).pin;
//
////         std_msgs::String temp;
//        if (io_button_num != 16)
//            ROS_ERROR_STREAM("Error setting of io_button_numuber = " << io_button_num);
//
//        int button_pressed = 0;
//        if (io_button) {
//            button_pressed = 1;
////                ROS_INFO_STREAM("io state: " << button_pressed);
////                temp.data = "teach_mode()\n"  ;
////                count_freedrive++;
////                pub_free_drive_.publish(temp);
//
//        }
////            else {
//////                temp.data = "end_freedrive_mode()\n";
////                temp.data = " ";
////            }
//        io_button_ = io_button;
////        ROS_INFO_STREAM("COUNT: "<< count_freedrive);
//    }

//    void URForceTeaching::subJointVoltageCB(sensor_msgs::JointState state) {
//        KDL::JntArray jntVol;
//        jntVol.resize(joint_size_);
//        int n = state.name.size();
//        for (int i = 0; i < joint_size_; ++i)//joint_names_
//        {
//            int x = 0;
//            for (; x < n; ++x)//state
//            {
//                if (state.name[x] == (prefix_ + joint_names_[i])) {
//                    jntVol(i) = state.effort[x];
//                    break;
//                }
//            }
//
//            if (x == n) {
//                ROS_ERROR_STREAM("Error,  joint name : " << prefix_ + joint_names_[i] << " , not found.  ");
//                return;
//            }
//        }
//        current_JntVol_ = jntVol;
////        ROS_INFO_STREAM("voltage="<<current_JntVol_(0)<<" "<<current_JntVol_(1)<<" "<<current_JntVol_(2)<<" "
////                                <<current_JntVol_(3)<<" "<<current_JntVol_(4)<<" "<<current_JntVol_(5));
//
//    }
    void URForceTeaching::subJointStateCB(sensor_msgs::JointState state) {
//    ROS_INFO("sub ur state");
        //get joint array
        KDL::JntArray jntArr;
        KDL::JntArray jntSpeed;
        KDL::JntArray jntCur;
        jntArr.resize(joint_size_);
        jntSpeed.resize(joint_size_);
        jntCur.resize(joint_size_);
        int n = state.name.size();
        for (int i = 0; i < joint_size_; ++i)//joint_names_
        {
            int x = 0;
            for (; x < n; ++x)//state
            {
                if (state.name[x] == (prefix_ + joint_names_[i])) {
                    jntArr(i) = state.position[x];
                    jntSpeed(i) = state.velocity[x];
                    jntCur(i) = state.effort[x];
                    break;
                }
            }

            if (x == n) {
                ROS_ERROR_STREAM("Error,  joint name : " << prefix_ + joint_names_[i] << " , not found.  ");
                return;
            }
        }
        current_JntArr_ = jntArr;
        current_JntSpeed = jntSpeed;
        current_JntCur_ = jntCur;
//            ROS_INFO_STREAM("voltage="<<jntSpeed(0)<<" "<<jntSpeed(1)<<" "<<jntSpeed(2)<<" "
//                                      <<jntSpeed(3)<<" "<<jntSpeed(4)<<" "<<jntSpeed(5));
        //fk
        p_fk_solver_->JntToCart(current_JntArr_, frame_wrist3_base_);    //frame_wrist3_base_ 为正运动学计算出的位姿
        double r11, p11, y11;
//            frame_wrist3_base_.M.GetRPY(r,p,y);   //base坐标系绕x y z 旋转得到末端tool1坐标系
////            ROS_INFO_STREAM("end_RPY: "<<r<<" "<<p<<" "<<y);
        KDL::Rotation world_base = KDL::Rotation::RPY(M_PI, 0, M_PI);  //world 到base坐标系
        (world_base * frame_wrist3_base_.M).GetRPY(r11, p11, y11);
//            ROS_INFO_STREAM("end_RPY: "<<r11<<" "<<p11<<" "<<y11);

        end_rpy_.data[0] = r11;
        end_rpy_.data[1] = p11;
        end_rpy_.data[2] = y11;  //相对于base坐标系的rpy


        geometry_msgs::Point end_point;
        end_point.x = frame_wrist3_base_.p.data[0];
        end_point.y = frame_wrist3_base_.p.data[1];
        end_point.z = frame_wrist3_base_.p.data[2];
        sub_end_position_.publish(end_point);


//            end_point_.data[0] =
//            ROS_INFO_STREAM("frame_wrist3_base_:\n"<< frame_wrist3_base_.M.data[0]<<" "<<frame_wrist3_base_.M.data[1]<<" "<<frame_wrist3_base_.M.data[2]<<"\n"
//                                    << frame_wrist3_base_.M.data[3]<<" "<<frame_wrist3_base_.M.data[4]<<" "<<frame_wrist3_base_.M.data[5]<<"\n"
//                                    << frame_wrist3_base_.M.data[6]<<" "<<frame_wrist3_base_.M.data[7]<<" "<<frame_wrist3_base_.M.data[8]<<"\n");

        end_point_.data[0] = end_point.x;
        end_point_.data[1] = end_point.y;
        end_point_.data[2] = end_point.z;



        bur_sub_ = true;
    }

    void URForceTeaching::subWrenchCB(geometry_msgs::WrenchStamped wrench) {
//    ROS_INFO("sub wrench");
        g_wrench_ = wrench.wrench;
        //repair for sensor installation
        current_wrench_.force.data[0] = wrench.wrench.force.x - repair_wrench_.force.data[0];
        current_wrench_.force.data[1] = wrench.wrench.force.y - repair_wrench_.force.data[1];
        current_wrench_.force.data[2] = wrench.wrench.force.z - repair_wrench_.force.data[2];
        current_wrench_.torque.data[0] = wrench.wrench.torque.x - repair_wrench_.torque.data[0];
        current_wrench_.torque.data[1] = wrench.wrench.torque.y - repair_wrench_.torque.data[1];
        current_wrench_.torque.data[2] = wrench.wrench.torque.z - repair_wrench_.torque.data[2];
//            ROS_INFO_STREAM("force current_wrench_: "<<current_wrench_.force[0]<<",  "<<current_wrench_.force[1]<<",  "<<current_wrench_.force[2]);

        //
        gravityRepair(current_wrench_);
//            ROS_INFO_STREAM("force current_wrench_ gravityRepair: "<<current_wrench_.force[0]<<",  "<<current_wrench_.force[1]<<",  "<<current_wrench_.force[2]);

        //
//            KDL::Wrench filtered ;
        wrenchFilter(current_wrench_, filtered);   //线性平滑滤波
        //
//            filtered = current_wrench_;
        if (bstate_change_ == true) {
            init_wrench_ = filtered;
            bstate_change_ = false;
        }
//            ROS_INFO_STREAM("force filtered: "<<filtered.force[0]<<",  "<<filtered.force[1]<<",  "<<filtered.force[2]);

        KDL::Vector force_EE = rot_FT_EE_ * filtered.force;
        KDL::Frame frame_EE_base = frame_base_world_ * frame_wrist3_base_;
        KDL::Vector force_base = frame_EE_base.M * force_EE;
//    ROS_INFO_STREAM("force in base: "<<force_base[0]<<",  "<<force_base[1]<<",  "<<force_base[2]);
        KDL::Vector torque_base = frame_EE_base.M * rot_FT_EE_ * filtered.torque;


        wrench_base_.force = frame_base_world_.M.Inverse() * force_base;
        wrench_base_.torque = frame_base_world_.M.Inverse() * torque_base;  //计算相对于base坐标系的力和力矩
        geometry_msgs::Wrench msg;
        msg.force.x = wrench_base_.force.data[0];
        msg.force.y = wrench_base_.force.data[1];
        msg.force.z = wrench_base_.force.data[2];
        msg.torque.x = wrench_base_.torque.data[0];
        msg.torque.y = wrench_base_.torque.data[1];
        msg.torque.z = wrench_base_.torque.data[2];
        pub_wrench_in_base_.publish(msg);

        std_msgs::Float64 msg_contact_force;
        msg_contact_force.data = -current_wrench_.force.data[2];
            pub_contact_force_.publish(msg_contact_force);

        // do force teaching

        // cancel do force_teaching
//            if (teaching_state_ != STOP) {
//                doForceTeaching(filtered);
//            }


        bwrench_sub_ = true;

    }

    void URForceTeaching::wrenchFilter(KDL::Wrench &input, KDL::Wrench &result) {
        if (bfirst_ef_wrench_) {
            ef_filter_wrench_ = input;
            bfirst_ef_wrench_ = false;
        } else {
            nh_.getParam("ef_wrench_factor", ef_wrench_factor_);
            ef_filter_wrench_ += ef_wrench_factor_ * (input - ef_filter_wrench_);
        }
        result = ef_filter_wrench_;
    }

    void URForceTeaching::twistFilter(KDL::Twist &input, KDL::Twist &result) {
        if (bfirst_ef_twist_) {
            ef_filter_twist_ = input;
            bfirst_ef_twist_ = false;
        } else {
            nh_.getParam("ef_twist_factor", ef_twist_factor_);
            ef_filter_twist_ += ef_twist_factor_ * (input - ef_filter_twist_);
        }
        result = ef_filter_twist_;
    }



    void URForceTeaching::gravityRepair(KDL::Wrench &wrench) {
        //transform gravity to sensor
        KDL::Frame frame_EE_base = frame_base_world_ * frame_wrist3_base_*frame_EE_wrist3_;;
//            KDL::Frame frame_EE_base = frame_wrist3_base_;
//            ROS_INFO_STREAM("frame_EE_base : \n"<< frame_EE_base.M.data[0]<<" "<<frame_EE_base.M.data[1]<<frame_EE_base.M.data[2]<<"\n"
//                                                     <<frame_EE_base.M.data[3]<<" "<<frame_EE_base.M.data[4]<<frame_EE_base.M.data[5]<<"\n"
//                                                     << frame_EE_base.M.data[6]<<" "<<frame_EE_base.M.data[7]<<frame_EE_base.M.data[8]<<"\n");
        KDL::Vector gs = (frame_EE_base.M * rot_FT_EE_).Inverse() * gravity_;
//            ROS_INFO_STREAM("gravity.force in sensor: "<<gs.data[0]<<", "<<gs.data[1]<<", "<<gs.data[2]);
//            ROS_INFO_STREAM("gravity.force in base  : "<<gravity_.data[0]<<", "<<gravity_.data[1]<<", "<<gravity_.data[2]);
        double l = -l1;   //重心位置
        //repair
//            l = 0.1;   // todo
        wrench.force -= gs;
        wrench.torque.data[0] += gs.data[1] * l;
        wrench.torque.data[1] -= gs.data[0] * l;
//            ROS_INFO_STREAM("gravity.torque in sensor: "<<wrench.torque[0]<<", "<<wrench.torque[1]<<", "<<wrench.torque[2]);


    }





    void URForceTeaching::record_force_sensor_repair() {
        std::ofstream file3;
        std::ostringstream oss;
        int as = 1;
        oss << as;
        std::string filename = oss.str() + "_record_Gravity";
        std::string file = "/home/gx/Polishing/src/force_tracking/config" + filename + ".txt";
        file3.open(file.c_str());
        file3 << gravity_.data[0] << " " << gravity_.data[1] << " " << gravity_.data[2] << " "
              << repair_wrench_.force.data[0] << " " << repair_wrench_.force.data[1] << " "
              << repair_wrench_.force.data[2] << " "
              << repair_wrench_.torque.data[0] << " " << repair_wrench_.torque.data[1] << " "
              << repair_wrench_.torque.data[2] << " "
              << l1;

        file3 << std::endl;
        file3.close();
    }

    void URForceTeaching::load_force_sensor_repair() {
        int row_Y = 1;
        int col_Y = 10;
        double repair_G[row_Y][col_Y];

        double **a;
        a = new double *[row_Y];
        for (int i = 0; i < row_Y; i++)
            a[i] = new double[col_Y];
        std::ostringstream oss;
        oss << 1;
        std::string filename = oss.str() + "_record_Gravity";
        std::string file = "/home/gx/Polishing/src/force_tracking/config" + filename + ".txt";

        std::ifstream fin(file);
        fin.trunc;
        for (int i = 0; i < row_Y; i++) {
            for (int j = 0; j < col_Y; j++) {
                fin >> a[i][j];
                repair_G[i][j] = a[i][j];

            }
        }
        fin.close();
        ROS_INFO_STREAM("gravity_:      " << repair_G[0][0] << " " << repair_G[0][1] << " " << repair_G[0][2]);
        ROS_INFO_STREAM("repair_force:  " << repair_G[0][3] << " " << repair_G[0][4] << " " << repair_G[0][5]);
        ROS_INFO_STREAM("repair_wrench: " << repair_G[0][6] << " " << repair_G[0][7] << " " << repair_G[0][8]);
        ROS_INFO_STREAM("l1:            " << repair_G[0][9]);
        for (int k = 0; k < 3; ++k) {
            gravity_.data[k] = repair_G[0][k];
            repair_wrench_.force.data[k] = repair_G[0][k + 3];
            repair_wrench_.torque.data[k] = repair_G[0][k + 6];
        }
        l1 = repair_G[0][9];
        ROS_INFO_STREAM("Force sensor calibration has been done");




    }

    void URForceTeaching::recordGravity() {
        //sub
        bur_sub_ = false;
        bwrench_sub_ = false;
        do {
            ros::spinOnce();
        } while (bur_sub_ == false || bwrench_sub_ == false);
        //up

        std::vector<double> gravity_up_joints;
        nh_.getParam("gravity_up_joints", gravity_up_joints);
        if (gravity_up_joints.size() != joint_size_) {
            ROS_ERROR("Wrong gravity_up_joints size for recording gravity!");
            return;
        }

        KDL::JntArray gravity_up_jnt(joint_size_);
        for (int i = 0; i < joint_size_; ++i) {
            gravity_up_jnt(i) = gravity_up_joints[i] * M_PI / 180;
        }

//                moveTo(gravity_up_jnt, 5, false);
//                moveto_joints(gravity_up_jnt,5);
        servoj_moveto(gravity_up_jnt, 5, true);
        ros::Duration(0.8).sleep();
        //record
        bwrench_sub_ = false;
        do {
            ros::spinOnce();
        } while (bwrench_sub_ == false);
        geometry_msgs::Wrench up_wrench = g_wrench_;


        //down
        std::vector<double> gravity_down_joints;
        nh_.getParam("gravity_down_joints", gravity_down_joints);
        if (gravity_down_joints.size() != joint_size_) {
            ROS_ERROR("Wrong gravity_down_joints size for recording gravity!");
            return;
        }

        KDL::JntArray gravity_down_jnt(joint_size_);
        for (int i = 0; i < joint_size_; ++i) {
            gravity_down_jnt(i) = gravity_down_joints[i] * M_PI / 180;
        }

//                moveTo(gravity_down_jnt, 3, false);
//                moveto_joints(gravity_down_jnt, 3);
        servoj_moveto(gravity_down_jnt, 3, true);
        ros::Duration(0.8).sleep();
        //record
        bwrench_sub_ = false;
        do {
            ros::spinOnce();
        } while (bwrench_sub_ == false);
        geometry_msgs::Wrench down_wrench = g_wrench_;

        gravity_.data[0] = (up_wrench.force.x - down_wrench.force.x) / 2;
        gravity_.data[1] = (up_wrench.force.y - down_wrench.force.y) / 2;
        gravity_.data[2] = (up_wrench.force.z - down_wrench.force.z) / 2;
        ROS_INFO_STREAM("gravity record value: " << gravity_.data[0] << ", " << gravity_.data[1] << ", "
                                                 << gravity_.data[2]);
        repair_wrench_.force.data[0] = (up_wrench.force.x + down_wrench.force.x) / 2;
        repair_wrench_.force.data[1] = (up_wrench.force.y + down_wrench.force.y) / 2;
        repair_wrench_.force.data[2] = (up_wrench.force.z + down_wrench.force.z) / 2;
//                ROS_INFO_STREAM("repair_wrench_.force record value: " << repair_wrench_.force.data[0] << ", "
//                                                                      << repair_wrench_.force.data[1] << ", "
//                                                                      << repair_wrench_.force.data[2]);
        repair_wrench_.torque.data[0] = (up_wrench.torque.x + down_wrench.torque.x) / 2;
        repair_wrench_.torque.data[1] = (up_wrench.torque.y + down_wrench.torque.y) / 2;
        repair_wrench_.torque.data[2] = (up_wrench.torque.z + down_wrench.torque.z) / 2;
//                ROS_INFO_STREAM("repair_wrench_.force record value: " << repair_wrench_.torque.data[0] << ", "
//                                                                      << repair_wrench_.torque.data[1] << ", "
//                                                                      << repair_wrench_.torque.data[2]);
//                // cal gravity position

        KDL::JntArray middle_jnt1;
        middle_jnt1.resize(6);
        std::vector<double> middle_jnt_;
        nh_.getParam("middle_jnt_", middle_jnt_);
        for (int y = 0; y < 6; ++y) {
            middle_jnt1(y) = middle_jnt_[y] * M_PI / 180;
        }

//                moveTo(middle_jnt1, 2.0, false);
//                moveto_joints(middle_jnt1, 2.0);
        servoj_moveto(middle_jnt1, 2, true);
        ros::Duration(1).sleep();
        bwrench_sub_ = false;
        do {
            ros::spinOnce();
        } while (bwrench_sub_ == false);

        double torque_x1 = current_wrench_.torque.data[1];
        l1 = torque_x1 / gravity_.data[2];
        ROS_INFO_STREAM("gravity position l1 =  " << l1 << " torque_y1 = " << torque_x1);


//            moveTo(gravity_down_jnt, 2, false);
        servoj_moveto(gravity_down_jnt, 2, true);
    }


    void URForceTeaching::moveto_first_point() {
        ROS_INFO("move to first point 1 ");
        // 以防逆解失败
        KDL::JntArray first_jnt_0;
        first_jnt_0.resize(6);
        double first_jnt_d[6] = {284.04, -80.31, -100.82, -89.28, 90.18, 59.06};
        for (int i = 0; i < 6; ++i) {
            first_jnt_0(i) = first_jnt_d[i] * M_PI / 180;
        }
        ros::spinOnce();
        servoj_moveto(first_jnt_0, 5,true);
        ros::Duration(0.5).sleep();

        KDL::Frame first_pose;
        first_pose.M.data[0] = 1;
        first_pose.M.data[1] = 0;
        first_pose.M.data[2] = 0;
        first_pose.M.data[3] = 0;
        first_pose.M.data[4] = 0;
        first_pose.M.data[5] = 1;
        first_pose.M.data[6] = 0;
        first_pose.M.data[7] = -1;
        first_pose.M.data[8] = 0;

        first_pose.p.data[0] = 0.007;
        first_pose.p.data[1] = 0.4252;
        first_pose.p.data[2] = 0.21;

        KDL::JntArray first_jnt;
        first_jnt.resize(6);
        ROS_INFO("move to first point 2 ");
        bur_sub_ == false;
        do {
            ros::spinOnce();
        } while (bur_sub_ == false);
        p_tracik_solver_->CartToJnt(current_JntArr_, first_pose, first_jnt);
        ROS_INFO_STREAM("current joints: " << current_JntArr_(0) << ", " << current_JntArr_(1) << ", "
                                           << current_JntArr_(2) << ", " << current_JntArr_(3) << ", "
                                           << current_JntArr_(4) << ", " << current_JntArr_(5));
        ROS_INFO_STREAM("first joints: " << first_jnt(0) << ", " << first_jnt(1) << ", " << first_jnt(2) << ", "
                                         << first_jnt(3) << ", " << first_jnt(4) << ", " << first_jnt(5));

//                moveTo(first_jnt, 2, false);
        servoj_moveto(first_jnt, 2,true);
        ros::Duration(0.5).sleep();

    }







    void URForceTeaching::moveto_joints(KDL::JntArray target_jnt, double time) {
//                ROS_INFO("move to joints");
        //运行前需要ros::spinOnce 来更新当前关节角
        bur_sub_ = false;
        do {
            ros::spinOnce();
        } while (bur_sub_ == false);

        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");

        for (int x = 0; x < 6; ++x) {
            p0.positions.push_back(current_JntArr_(x));
            p0.velocities.push_back(0);
        }
        p0.time_from_start = ros::Duration(0);
        g.trajectory.points.push_back(p0);

        for (int x = 0; x < 6; ++x) {
            p1.positions.push_back(target_jnt(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_->sendGoal(g);
        client_->waitForResult(ros::Duration());
    }

    void URForceTeaching::transform_Vector2R(KDL::Vector xyz1, KDL::Rotation &R_1) {
        Eigen::VectorXd xn(3), yn(3);
        yn << xyz1.data[0], xyz1.data[1], xyz1.data[2];
        xn << 1, 0, -yn(0) / yn(2);
        yn.normalize();
        xn.normalize();
        double zn[3] = {0, 0, 0};
        zn[0] = xn(1) * yn(2) - xn(2) * yn(1);
        zn[1] = -xn(0) * yn(2) + xn(2) * yn(0);
        zn[2] = xn(0) * yn(1) - xn(1) * yn(0);// 叉乘
        R_1.data[0] = xn(0);
        R_1.data[1] = yn(0);
        R_1.data[2] = zn[0];
        R_1.data[3] = xn(1);
        R_1.data[4] = yn(1);
        R_1.data[5] = zn[1];
        R_1.data[6] = xn(2);
        R_1.data[7] = yn(2);
        R_1.data[8] = zn[2];


    }

    void URForceTeaching::speedj_moveto(KDL::JntArray target, double time, bool wait_for_D) {
        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");

        for (int x = 0; x < 6; ++x) {
            p0.positions.push_back(current_JntArr_(x));
            p0.velocities.push_back(0);
        }
        p0.time_from_start = ros::Duration(0);
        g.trajectory.points.push_back(p0);

        for (int x = 0; x < 6; ++x) {
            p1.positions.push_back(target(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_speedj_->sendGoal(g);
        if (wait_for_D)
            client_speedj_->sendGoalAndWait(g);
        else
            client_speedj_->sendGoal(g);
//             client_speedj_->waitForResult(ros::Duration());

    }


    void URForceTeaching::servoj_moveto(KDL::JntArray target, double time, bool wait_for_D) {
        trajectory_msgs::JointTrajectoryPoint p0;
        trajectory_msgs::JointTrajectoryPoint p1;
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        g.trajectory.joint_names.push_back("shoulder_pan_joint");
        g.trajectory.joint_names.push_back("shoulder_lift_joint");
        g.trajectory.joint_names.push_back("elbow_joint");
        g.trajectory.joint_names.push_back("wrist_1_joint");
        g.trajectory.joint_names.push_back("wrist_2_joint");
        g.trajectory.joint_names.push_back("wrist_3_joint");

        for (int x = 0; x < 6; ++x) {
            p0.positions.push_back(current_JntArr_(x));
            p0.velocities.push_back(0);
        }
        p0.time_from_start = ros::Duration(0);
//        g.trajectory.points.push_back(p0);

        for (int x = 0; x < 6; ++x) {
            p1.positions.push_back(target(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_servoj_->sendGoal(g);
        if (wait_for_D)
            ros::Duration(time).sleep();

    }



    void URForceTeaching::polishing_tracking() {
        //load trajectory,desired force
        int start_row = 1100;
        int row_Y = 3137;
        int num_point = row_Y - start_row + 1;
        double start_time = 0,time_rate = 1;
        load_data_file(start_row,time_rate);                    //load trajectory to array Y;
        //move to start point
        moveto_first_trajectory_point(start_row);     //move to first point of trajectory

        // while loop to track force. Record force and end position

        end_trajectory_state = false;
        KDL::JntArray ref_joints;
        uint64_t nsec0 = ros::Time::now().toNSec();
        double time_flag = 0;
        ros::Rate loop_rate(125);
        contact_adjust.initialize();
        while (ros::ok() & end_trajectory_state == false) {

            ros::spinOnce();

            // 时间-关节角度序列插值
            ref_joints = current_JntArr_;
            uint64_t time_now = ros::Time::now().toNSec() - nsec0;
            double time_now_d = time_now / 1e9;                        //current time

//            ROS_INFO_STREAM("current time " << time_now_d<<"     end point.x"<<end_point_.data[0]);
            interpolate_traj_test(start_row, row_Y, time_now_d, ref_joints);  //按照时间插值
            std_msgs::Float64 pub_ref_f;
            pub_ref_f.data = ref_force_;
            pub_ref_force_.publish(pub_ref_f);

            servoj_moveto(ref_joints,0.008,false);   //

            //record
            std::ofstream file3;
            std::ostringstream oss;
            oss << name_suffix;
            std::string filename = oss.str() + "_Polishing_tracking";
            std::string file = "/home/gx/Polishing/src/force_tracking/record/Polishing/" + filename + ".txt";
            file3.open(file.c_str(), std::ios::app);
            file3 << time_now_d << " ";
            file3 << ref_force_<< " " << - current_wrench_.force.data[2]<< " " << std_f_<< " "
                  << end_point_.data[0]<< " " << end_point_.data[1] << " " <<end_point_.data[2]
                  << std::endl;
            file3.close();

            loop_rate.sleep();

        }




    }

    void URForceTeaching::test_servoj() {
        std::vector<double> gravity_down_joints;
        nh_.getParam("gravity_down_joints", gravity_down_joints);
        KDL::JntArray gravity_down_jnt(joint_size_);
        for (int i = 0; i < joint_size_; ++i) {
            gravity_down_jnt(i) = gravity_down_joints[i] * M_PI / 180;
        }
        ROS_INFO_STREAM("t0 ");
        servoj_moveto(gravity_down_jnt, 3, true);
        ros::spinOnce();

        ROS_INFO_STREAM("t1 ");
        bur_sub_ = false;
        do{
            ros::spinOnce();
        }while(!bur_sub_);
        KDL::Frame original_P = frame_wrist3_base_;
        ros::Rate loop_rate(125);
        uint64_t nsec0 = ros::Time::now().toNSec();
        while(ros::ok()){
            ros::spinOnce();
            double sum_force_all =std::sqrt(current_wrench_.force.data[0]*current_wrench_.force.data[0]+
                                            current_wrench_.force.data[1]*current_wrench_.force.data[1]+
                                            current_wrench_.force.data[2]*current_wrench_.force.data[2]);
            if (sum_force_all > 30) {
                ROS_INFO_STREAM("force is too big !!!    stop moving");
                ros::Duration(100).sleep();
            }
            uint64_t time_now = ros::Time::now().toNSec() - nsec0;
            double time_now_d = time_now / 1e9;
            KDL::Frame p1 = original_P;
            KDL::JntArray q1;
            q1.resize(6);
            p1.p.data[2] = original_P.p.data[2]+ 0.05*sin(2*M_PI/5*time_now_d);
            p_tracik_solver_->CartToJnt(current_JntArr_,p1,q1);
            servoj_moveto(q1,0.008,false);

            //record
            std::ofstream file3;
            std::ostringstream oss;
            oss << name_suffix;
            std::string filename = oss.str() + "_position_servoj_tracking";
            std::string file = "/home/gx/Polishing/src/force_tracking/record/" + filename + ".txt";
            file3.open(file.c_str(), std::ios::app);
            file3 << time_now_d << " ";
            file3 << p1.p.data[0]<< " " <<p1.p.data[1] << " " << p1.p.data[2]<< " "
                  << end_point_.data[0]<< " " << end_point_.data[1] << " " <<end_point_.data[2];
            file3 << std::endl;
            file3.close();
            loop_rate.sleep();
        }








    }


}