// ros
#include "ros/ros.h"

// eigen
#include <eigen3/Eigen/Dense>

// pid
#include "PID_lib/pid.h"

// msg
#include "odom_uav/ukfData.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt8.h"
#include <dji_sdk/api_ctrl_data.h>

//YU YUN
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"

using namespace std;
using namespace ros;

// Ros
ros::Subscriber ukfsub, mission_status_sub;
ros::Publisher ctrlpub, yaw_cali_pub;
ros::Publisher debug_pub;

// target pose
float targetPosition[3] = {0.0, 0.0, 0.0};
float targetYaw = 0.0;
float flight_height = 120.0;

// PID related variables
PID *ctrl_x;
PID *ctrl_y;
PID *ctrl_z;
PID *ctrl_yaw;

double Kp_pos_x;
double Kd_pos_x;
double Ki_pos_x;

double Kp_pos_y;
double Kd_pos_y;
double Ki_pos_y;

double Kp_pos_z;
double Kd_pos_z;
double Ki_pos_z;

double Kp_yaw;
double Ki_yaw;
double Kd_yaw;

int countX = 0,countY = 0,countZ = 0, countYaw = 0;
bool tracking = true, hovering = false, aiming =false;
double pid_gain = 1.0;

double controlLimit, controlLimitVel, controlLimitYawrate;

float target_velX, target_velY, target_velZ;
float yaw_eb , target_yaw_rate;

static vector3f v_et, v_bb ; // final control variables
static vector4f controlInput, q_eb;
static matrix3f R_eb, R_be, R_bc;

bool is_debug_on;

float p_pos_x, i_pos_x, d_pos_x, error_pos_x;
float p_pos_y, i_pos_y, d_pos_y, error_pos_y;
float p_pos_z, i_pos_z, d_pos_z, error_pos_z;

float p_yaw, i_yaw, d_yaw, error_yaw;

// function
void ukfCallback(const odom_uav::ukfData& ukf);
void getYaw(vector4f q_tc, float &yaw);
void autoAim(float& ctrlZ, float& ctrlYaw,
             int& cX, int& cY, int& cZ, int & cYaw,
             float errorX, float errorY, float errorZ ,float errorYaw);

//#include "pid_controller.h"

int main (int argc, char** argv)
{
    // ros init and parameters retrieve
    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    is_debug_on = true;

    nh.param("is_debug_on", is_debug_on, true);

    // ctrl x
    nh.param("Kp_pos_x", Kp_pos_x, 0.74);
    nh.param("Ki_pos_x", Ki_pos_x, 0.02);
    nh.param("Kd_pos_x", Kd_pos_x, 0.41);

    // ctrl y
    nh.param("Kp_pos_y", Kp_pos_y, 0.74);
    nh.param("Ki_pos_y", Ki_pos_y, 0.02);
    nh.param("Kd_pos_y", Kd_pos_y, 0.41);

    // ctrl z
    nh.param("Kp_pos_z", Kp_pos_z, 0.40);
    nh.param("Ki_pos_z", Ki_pos_z, 0.00);
    nh.param("Kd_pos_z", Kd_pos_z, 0.00);

    // ctrl yaw
    nh.param("Kp_yaw", Kp_yaw, 0.800);
    nh.param("Ki_yaw", Ki_yaw, 0.010);
    nh.param("Kd_yaw", Kd_yaw, 1.000);

    //////////
    nh.param("pid_gain", pid_gain, 1.0);

    nh.param("controlLimit", controlLimit, 1.0);
    nh.param("controlLimitVel", controlLimitVel, 0.5);
    nh.param("controlLimitYawrate", controlLimitYawrate, 10.0);
    ////////////////////////////////////////////////////////

    //init pid controllers
    ctrl_x = new PID(Kp_pos_x, Ki_pos_x, Kd_pos_x, -200, 200, -950, 950);
    ctrl_y = new PID(Kp_pos_y, Ki_pos_y, Kd_pos_y, -200, 200, -950, 950);
    ctrl_z = new PID(Kp_pos_z, Ki_pos_z, Kd_pos_z, -200, 200, -950, 950);

    ctrl_yaw = new PID(Kp_yaw, Ki_yaw, Kd_yaw, -200, 200, -150, 150);

    ctrl_x -> set_point(targetPosition[0]);
    ctrl_y -> set_point(targetPosition[1]);
    ctrl_z -> set_point(targetPosition[2]);

    ctrl_yaw -> set_point(targetYaw);

    ctrlpub = nh.advertise<dji_sdk::api_ctrl_data>("/sdk_request_ctrl",10);

    ukfsub  = nh.subscribe("/ukf_data", 10, ukfCallback);

    cout<< "uav controller start!"<<endl;
    ros::spin();
    cout<< "uav controller shutdown!"<<endl;
    return 1;
}

void ukfCallback(const odom_uav::ukfData& ukf)
{
    cout<<endl<< "=====================================" << endl;
    cout<< "call pid ctrl!"<<endl;

    dji_sdk::api_ctrl_data outMsg;

    float delta_t = ukf.dt;

    flight_height = ukf.flight_height;

    if(ukf.isctrl ==1)
    {

        // ready to generate control info

        // get v_et from t_et
        target_velX = ctrl_x -> update(ukf.xyz.x, delta_t, &error_pos_x, &p_pos_x, &i_pos_x, &d_pos_x);   //control roll
        target_velY = ctrl_y -> update(ukf.xyz.y, delta_t, &error_pos_y, &p_pos_y, &i_pos_y, &d_pos_y);   //control pitch
        target_velZ = ctrl_z -> update(ukf.xyz.z - flight_height, delta_t, &error_pos_z, &p_pos_z, &i_pos_z, &d_pos_z);   //control vel

        cout<<"t_et x| "<<ukf.xyz.x<<" y| "<<ukf.xyz.y<<" z| "<<ukf.xyz.z<<endl;
        cout<<"v_et x| "<<target_velX<<" y| "<<target_velY<<" z| "<<target_velZ<<endl;

        v_et[0] = target_velX;
        v_et[1] = target_velY;
        v_et[2] = target_velZ;

        // transform acc_et to acc_bt
        q_eb[0] = ukf.q.w;
        q_eb[1] = ukf.q.x;
        q_eb[2] = ukf.q.y;
        q_eb[3] = ukf.q.z;

        getYaw(q_eb, yaw_eb);

        //float delta_yaw;
        //delta_yaw = ukf.tgt_yaw - yaw_eb;
        //target_yaw_rate = - ctrl_yaw-> update(delta_yaw, delta_t, &error_yaw, &p_yaw, &i_yaw, &d_yaw);

        //cout<< "targetYaw| "<< targetYaw
        //    << " Yaw| "	    << yaw_eb
        //    << " ctrl| "    << target_yaw_rate
        //    << " error| "   << error_yaw
        //    <<endl;


        quat_to_DCM(R_eb, q_eb);

        /// R_be
        get_matrix3f_transpose(R_be, R_eb);

        // v_bb = R_be * v_et
        matrix3f_multi_vector3f(v_bb, R_be, v_et);

        cout<<"v_bb x| "<<v_bb[0]<<" y| "<<v_bb[1]<<" z| "<<v_bb[2]<<endl;

        // input vel
        controlInput[0] = pid_gain* v_bb[0]/100; // body x , roll
        controlInput[1] = pid_gain* v_bb[1]/100; // body y , pitch
        controlInput[2] = 0;//pid_gain* target_yaw_rate;   // 0 yaw control
        controlInput[3] = pid_gain* v_bb[2]/100;    // yaw control

        cout<<"ctrl data"
           <<" x| "<<controlInput[0]
          <<" y| "<<controlInput[1]
         <<" yaw| "<<controlInput[2]
        <<" z| "<<controlInput[3]
        <<endl;

        // constrain control input
        if (controlInput[0] > controlLimit)
            controlInput[0] = controlLimit;
        if (controlInput[0] < -controlLimit)
            controlInput[0] = -controlLimit;

        if (controlInput[1] > controlLimit)
            controlInput[1] = controlLimit;
        if (controlInput[1] < -controlLimit)
            controlInput[1] = -controlLimit;

        if (controlInput[2] > controlLimitYawrate)
            controlInput[2] = controlLimitYawrate;
        if (controlInput[2] < -controlLimitYawrate)
            controlInput[2] = -controlLimitYawrate;

        if (controlInput[3] > controlLimitVel)
            controlInput[3] = controlLimitVel;
        if (controlInput[3] < -controlLimitVel)
            controlInput[3] = -controlLimitVel;

        //autoAim(controlInput[2],controlInput[3], countX, countY, countZ, countYaw,
        //        error_pos_x, error_pos_y, error_pos_z, error_yaw);

        // output msg
        //outMsg.header.stamp =ros::Time::now();
        //outMsg.header.frame_id = "pid_controller";

        outMsg.horiz_mode = 01;
        outMsg.vert_mode = 00;
        outMsg.yaw_mode = 1;
        outMsg.level_frame = 01;   //North Ground Earth

        outMsg.roll_or_x  = controlInput[0];    //roll
        outMsg.pitch_or_y = controlInput[1];   //pitch
        outMsg.yaw        = controlInput[2];          //yaw_rate
        outMsg.thr_z      = controlInput[3];        //vel

        ctrlpub.publish(outMsg);

    }
    else if(ukf.isctrl == 0)
    {
        ctrl_x->reduceIntegral(0);
        ctrl_y->reduceIntegral(0);
        ctrl_yaw->reduceIntegral(0);
        ctrl_z->reduceIntegral(0);
    }

}
/*********
 *
 *
 *
 */
void getYaw(vector4f q_tc, float &yaw)
{
    float yaw_rad, pitch_rad, roll_rad;
    float yaw_degree, pitch_degree, roll_degree;

    quat_to_eular(&yaw_rad, &pitch_rad, &roll_rad, q_tc);

    pitch_degree = pitch_rad * 180.0f/ C_PI_F;
    roll_degree  = roll_rad  * 180.0f/ C_PI_F;
    yaw_degree   = yaw_rad   * 180.0f/ C_PI_F;

    yaw = yaw_degree;

    cout<< "now_yaw  | "<< yaw_degree <<endl
        << "now_pitch| "<< pitch_degree <<endl
        << "now_roll | "<< roll_degree <<endl;
}

void autoAim(float& ctrlZ, float& ctrlYaw,
             int& cX, int& cY, int& cZ, int& cYaw,
             float errorX, float errorY, float errorZ, float errorYaw)
{
    // auto aiming
    if (tracking)
    {
        if ( abs(errorZ) < 7) cZ++;
        if ( cZ > 15 && abs(errorZ) < 5)
        {
            hovering = true;
            tracking = false;
        }
        cout<< "Tracking!!!!!!!!!!"<<endl;
        cout<< "errorZ: "<< errorZ<< endl;
        cout<< "40! :" <<cZ <<endl;
    }

    if ( hovering)
    {
        ctrlZ = 0;
        if ( abs(errorX)<7) cX++;
        if ( abs(errorY)<7) cY++;
        if ( abs(errorYaw)<1) cYaw++;
        if ( cYaw > 200 && abs(errorYaw) <1) ctrlYaw = 0;
        if ( cX > 300 && cY > 300 && abs(errorX) < 1 && abs(errorY) < 1 && ctrlYaw == 0)
        {
            hovering = false;
            aiming = true;
        }
        cout<< "Hovering!!!!!!!!!!!"<<endl;
        cout<< "errorX: "<< errorX<<" | "<<"errorY: "<<errorY <<"| errorYaw: "<<errorYaw<<endl;
        cout<< "300! :"<< cX << " | "<<cY <<" | "<< cYaw <<endl;
    }
    if ( aiming)
    {
        cout<< "aiming!!!!!!!!!!!!"<<endl;

        //ctrlZ = -0.70;
    }
}
