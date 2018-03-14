#ifndef UR_FORCE_TEACHING_HPP
#define UR_FORCE_TEACHING_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <std_msgs/Int32.h>
#include <trac_ik/trac_ik.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>



#include "ur_msgs/IOStates.h"
//#include "../../simulink2C/constant_contact/force_loop.h"
//#include "../../simulink2C/angular_vel_filter/angular_vel_filter.h"
//#include "../../simulink2C/vel_driver/choose_vel_pid2C.h"
#include "../../simulink2C/tracking/force_loop.h"



    namespace teaching {

        class URForceTeaching {

          typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> Client;

            enum TeachingState {
                STOP = 0,
                TRANSLATION = 1,
                ROTATION = 2,
                FREE = 3,
                YZPLANE = 4,
                ZCONTACT = 5,
                ZCONTACT_DEMO = 6
            };

            enum RecordingState {
                STOP_RECORD = 0,
                START_RECORD = 1
            };

        public:
            URForceTeaching(ros::NodeHandle nh, const std::string &urdf_param, std::string &prefix);

            ~URForceTeaching() {
                delete client_servoj_;
                delete p_fk_solver_;
                delete p_jac_solver_;
                delete p_tracik_solver_;
            }

            bool init();


            void recordGravity();
            void run();

            void interpolate_traj_test(int start_row,int row_Y,double time_now,KDL::JntArray &ref_joints); //
            void interpolate_traj(int start_row,int row_Y,double time_now,KDL::JntArray &jt);

            void load_data_file(int start_row,double time_rate);   //load Y of the learning result.

            void moveto_first_point();
            void moveto_joints(KDL::JntArray target_jnt, double time);
            void speedj_moveto(KDL::JntArray target,double time,bool wait_for_D);
            void servoj_moveto(KDL::JntArray target,double time,bool wait_for_D);
            void test_freeDrive();
            void search_hole(double delta_sita_offset, double delta_xy_offset);
            void vel_move_driver(KDL::JntArray q0);
            void test_vel_driver();
            void record_force_sensor_repair();
            void load_force_sensor_repair();
            void stop_pub_vel();
            void force_mode_record();
            void test_servoj();
            void auto_peg_in_hole();

            void eigen_learning();


            void polishing_tracking();

//            constant_contact::force_PID_loop force_loop_;
//            torque_tracking torque_sin_tracking_;
//            Vel_filter::angular_vel_filter_class Wxy_filter_;
//            velocity_pid_loop vel_driver_;
        force_PID_loop contact_adjust;


        private:
            void subJointStateCB(sensor_msgs::JointState state);
//            void subJointVoltageCB(sensor_msgs::JointState state);
            void subWrenchCB(geometry_msgs::WrenchStamped wrench);
//            void subTeachingStateCB(std_msgs::Int32 state);
            void sub_io_state(ur_msgs::IOStates iostate);



            double generate_uniform_distr(double range_from, double range_to );

            bool io_button_;
            bool write_data;

            void wrenchFilter(KDL::Wrench &input, KDL::Wrench &result);
            void twistFilter(KDL::Twist &input, KDL::Twist &result);
            void gravityRepair(KDL::Wrench &wrench);










            void recordingThread();
            void subRecordingStateCB(std_msgs::Int32 state);
            void sub_end_position(geometry_msgs::Point end_point);
//            void subWriteData(std_msgs::Int32 state);

            void  moveto_first_trajectory_point(int start_row);

            void transform_Vector2R(KDL::Vector xyz1,KDL::Rotation &R_1);



        private:
            bool force_tracking_state_;
            ros::Subscriber sub_recording_state_;


//            ros::Subscriber sub_write_data;

            std::ofstream record_fstream_;

            ros::NodeHandle nh_; //???
            std::string prefix_;

            int joint_size_;
            std::vector<std::string> joint_names_; //定义一个大小可变的数组保存字符串

            KDL::JntArray current_JntArr_;
            KDL::JntArray current_JntSpeed;
            KDL::JntArray current_JntCur_;
            KDL::JntArray current_JntVol_;
            //limit
            KDL::JntArray max_jnt_;
            KDL::JntArray min_jnt_;
            KDL::JntArray vel_limit_;
            KDL::Vector end_rpy_;
            KDL::Vector end_point_;

            ros::Publisher pub_vel_;
            ros::Publisher pub_wrench_in_base_;
            ros::Publisher pub_speed_;
            ros::Publisher pub_contact_force_;
            ros::Publisher pub_filtered_contact_force_;
            ros::Publisher pub_angular_vel_;

            ros::Publisher sub_end_position_;
            ros::Publisher pub_ref_force_;
//            ros::Publisher pub_effort_;


            int  name_suffix ;
            int row_Y ;
            int col_Y ;
            bool end_trajectory_state;


            ros::Subscriber sub_teaching_state_;
            ros::Subscriber sub_joint_state_;
//            ros::Subscriber sub_joint_voltage_;
            ros::Subscriber sub_wrench_;
            ros::Subscriber sub_io_state_;
            bool bur_sub_;
            bool bwrench_sub_;

            KDL::Chain chain_;
            TRAC_IK::TRAC_IK *p_tracik_solver_;
            KDL::ChainFkSolverPos_recursive *p_fk_solver_;//KDL正运动学求解器
            KDL::ChainJntToJacSolver *p_jac_solver_;
            Client *client_;  //???
            Client *client_speedj_;
            Client *client_servoj_;

            int teaching_state_;
            bool bstate_change_;

            KDL::Wrench init_wrench_;
            KDL::Wrench repair_wrench_;
            KDL::Wrench current_wrench_;
            KDL::Wrench filtered;
            KDL::Wrench wrench_base_;
            geometry_msgs::Wrench g_wrench_;

            KDL::Vector gravity_;
            KDL::Rotation rot_FT_EE_;
            KDL::Frame frame_EE_wrist3_;
            KDL::Frame frame_wrist3_base_;//fk result
            KDL::Frame frame_base_world_;

            std::vector<double> xyz_delta_velocity_;
            std::vector<double> xyz_delta_position_;
            std::vector<double> rot_delta_velocity_;
            std::vector<double> pid;
            std::vector<double> force_PID_data;
            std::vector<double> pid_vel;
            uint64_t nsec0_;
            int start_row_;
            double time_flag_;

//            double test_angle;


            double plane_angle ;
            double delta_time_;
            Eigen::VectorXd current_cart_vel_;
            Eigen::VectorXd  vel_err_;
            double translation_acc_;
            double rotation_acc_;

            //exponential-filter
            double ef_wrench_factor_;///<range 0~1
            bool bfirst_ef_wrench_;
            KDL::Wrench ef_filter_wrench_;
            double ef_twist_factor_;///<range 0~1
            double ef_joint_factor_;
            double PID_multiply_;
            bool bfirst_ef_twist_;
            KDL::Twist ef_filter_twist_;
            KDL::JntArray ef_filter_joint_;
            double l1;
            double tangential_speed_;
            double sum_delta_f2x_;
            double ref_force_;
            double std_f_;

            double Y[3137][10];
            double mean_[4];
            double Sigma_[4][4];
            double spring_delta_x_;







        };
    }




#endif // UR_FORCE_TEACHING_HPP
