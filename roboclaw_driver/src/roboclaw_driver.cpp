/*
 * Driver for the roboclaw USB motor controller
 *
 * Copyright (c) 2012-2014 Ricardo Garro
 * Copyright (c) 2014 Universitaet Bremen, Institute for Artificial Intelligence - Prof. Michael Beetz
 *
 * Authors: Ricardo Garro, Alexis Maldonado <amaldo@cs.uni-bremen.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */



#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include <string.h>
#include "std_msgs/String.h"

#include <stdio.h>
#include <math.h>



#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>                          // odom
#include <geometry_msgs/Twist.h>                        // cmd_vel


#define REPLY_SIZE 32

//These will contain -1 if the sign should be inverted
int left_motor_sign_  =1;
int right_motor_sign_ =1;


cereal::CerealPort device;
char reply[REPLY_SIZE];
const float cereal_timeout=0.1;


//Variables that describe physical details of the robot
//These values get overriden by parameters in main()
double tortugabot_wheel_diameter = 0.125;  // 10cm wheel is really about 97mm
double tortugabot_wheel_to_wheel_distance = 0.357;  //from the center of one wheel to the other, in meters
//Devantech EMG49 has 980 encoder ticks per shaft turn
//Devantech EMG30 has 360 encoder ticks per shaft turn
int tortugabot_ticks_per_turn = 980;
int tortugabot_pps_max = 2300; //maximum number of ticks/s when running with duty cycle of 100%
int tortugabot_pid_p = 65000;
int tortugabot_pid_i = 32768;
int tortugabot_pid_d = 16384;

unsigned char g_roboclaw_address = 128;

const double PI = 3.141592;

ros::Time last_cmd_received;
ros::Duration half_seconds(0.2);



//Get the checksum as calculated in the RoboClaw manual
unsigned char get_checksum(unsigned char *data, unsigned int length){
    unsigned char checksum = 0;

    unsigned int i = 0;

    for (i=0; i < length; ++i) {
        checksum += data[i];
    }

    checksum &= 0x7f;

    return(checksum);

}

//clamp the input variable to +-limit
int clamp_value(int input, int limit)
{
    if (input > limit) {
        input = limit;
    } else if (input < -limit) {
        input = -limit;
    }
    return(input);
}

//Function for sending simple commands that don't need a checksum
void write_to_roboclaw_2(unsigned char command)
{
    unsigned char address = g_roboclaw_address;

    unsigned char cmd[2];

    cmd[0] = address;
    cmd[1] = command;

    device.write((const char*)&cmd, 2);

}

//Function for sending simple commands with one-byte parameter
void write_to_roboclaw(unsigned char command, unsigned char bytevalue)
{
    unsigned char address = g_roboclaw_address;
    unsigned char checksum = (address + command + bytevalue) & 0x7F;

    unsigned char cmd[4];

    cmd[0] = address;
    cmd[1] = command;
    cmd[2] = bytevalue;
    cmd[3] = checksum;

    device.write((const char*)&cmd, 4);

}


void set_vel(int motor, int vel)
{

    unsigned char command;
  
    if (motor == 1) {
        command = 0;
    } else {
        command = 4;
    }

    if (vel > 255) {
        ROS_ERROR("Tried to command vel bigger than 255");
    }

    write_to_roboclaw(command, vel);
}


//Function to read the encoders. Important, called at the frequency of the main loop
int Read_Quadrature_M(int motor)
{
    if (motor > 2) {
        ROS_ERROR("Tried to read encoder from invalid motor. Returning 0");
        return(0);
    }

    unsigned char command;
    static int old_enc[2] = {0,0};

    if (motor == 1) {
        command = 16;
    } else {
        command = 17;
    }



    int gave_up_on_read = 0;

    const int num_retries = 3;
    unsigned char checksum_from_roboclaw;
    unsigned int i = 0;

    for (i=0; i < num_retries; ++i) {
        write_to_roboclaw_2(command);

        try {
            device.read(reply, 6, cereal_timeout);
        } catch(cereal::TimeoutException& e) {
            ROS_ERROR("Read_Quadrature_M: Timeout while reading!");
            device.flush();
            continue;

        }

        checksum_from_roboclaw = (g_roboclaw_address + command + ((unsigned char)(reply[0])) + ((unsigned char)(reply[1])) + ((unsigned char)(reply[2])) + ((unsigned char)(reply[3])) + ((unsigned char)(reply[4])) ) & 0x7F;


        if (checksum_from_roboclaw == (unsigned char)(reply[5])) {
            break;
        } else if (i == (num_retries - 1)) {
            //time to give up
            ROS_ERROR("Could not read encoder %d correctly. Returning old data. Odom will suffer a bit.", motor);
            gave_up_on_read = 1;
            device.flush();
            break;
           
        } else {
            //Still retrying
            //try clearing the serial port buffer
            device.flush();
        }

    }

    int counter = 0;
    if (gave_up_on_read == 0) {
        //it was a good read -> Parse the bytes in reply

        //if it gets here, the read was successful and the data is in reply[]
        unsigned char *counter_x;
        counter_x = reinterpret_cast<unsigned char *>(&counter);
        counter_x[0] = reply[3];
        counter_x[1] = reply[2];
        counter_x[2] = reply[1];
        counter_x[3] = reply[0];

        //save the counter
        old_enc[motor] = counter;

        //reply[4] is the status byte (bit0 = underflow occurred, bit1=direction, bit2 = overflow occurred)
        /*
        unsigned char status_byte = reply[4];
        int underflow = status_byte & 0x01;
        int direction = (status_byte & 0x02) >> 1;
        int overflow = (status_byte & 0x04) >> 2;
        */
        //FIXME: Deal with underflow and overflow

        //printf("motor = %d counter = %d under = %d  direction = %d  overflow = %d\n", motor, counter, underflow, direction, overflow);

    } else {
        //gave up on reading encoder
        //should return old values to minimize damage
        counter = old_enc[motor];
    }

    return(counter);


}


//FIXME: switch to write_to_roboclaw function call
void reset_counts()
{
    unsigned char address = 0;
    unsigned char command = 0;
    unsigned char bytevalue = 0;
    unsigned char checksum = 0;

    address = 128;
    command = 20;
    checksum = (address + command + bytevalue) & 0x7F;

    unsigned char cmd[3];

    cmd[0] = address;
    cmd[1] = command;
    cmd[2] = checksum;

    device.write((const char*)&cmd, 3);
}


//FIXME: Check this function
int  Read_Speed_M(int motor)
{
    unsigned char address;
    unsigned char command;
    unsigned char bytevalue;
    unsigned char checksum;


    address = 128;
    if (motor == 1) {
        command = 18;
    } else {
        command = 19;
    }

    bytevalue= 0;
    checksum = (address+command+bytevalue) & 0x7F;

    //  printf ("address: %d \n", address);
    //  printf ("command: %d \n", command);
    //  printf ("bytevalue: %d \n", bytevalue);
    //  printf ("checksum: %d \n", checksum);

    unsigned char cmd[4];

    cmd[0] = address;
    cmd[1] = command;
    cmd[2] = bytevalue;
    cmd[3] = checksum;

    device.write((const char*)&cmd, 4);


    try{ device.read(reply, 6, cereal_timeout); }
    catch(cereal::TimeoutException& e)
    {

        ROS_ERROR("Timeout!");
    }

    unsigned char checksum_send_roboclow;


    if (motor == 1) {
        checksum_send_roboclow= (128+18+((unsigned char)(reply[0]))+((unsigned char)(reply[1]))+ ((unsigned char)(reply[2]))+((unsigned char)(reply[3]))+((unsigned char)(reply[4])))& 0x7F;
    }
    else {
        checksum_send_roboclow= (128+19+((unsigned char)(reply[0]))+((unsigned char)(reply[1]))+ ((unsigned char)(reply[2]))+((unsigned char)(reply[3]))+((unsigned char)(reply[4])))& 0x7F;
    }


    //checksum_send_roboclow= (128+16+((unsigned char)(reply[0]))+((unsigned char)(reply[1]))+ ((unsigned char)(reply[2]))+((unsigned char)(reply[3]))+((unsigned char)(reply[4])))& 0x7F;
    //printf("checksum_calc = %d \n", checksum_send_roboclow);
    //printf("Reply 5 = %d \n",((unsigned char) (reply[5])));


    int counter;
    unsigned char *counter_x;
    counter_x = reinterpret_cast<unsigned char *>(&counter);
    counter_x[0] = reply[3];
    counter_x[1] = reply[2];
    counter_x[2] = reply[1];
    counter_x[3] = reply[0];



    if ( (unsigned char)(reply[5]) == checksum_send_roboclow ) {
        if (motor==1) {
            if (reply[4]==0) {
                printf("Speed M1= %d, forward  \n", counter);
                return  counter;
            }
            else {
                printf("Speed M1= %d, backward  \n", counter);
                return  counter;
            }
        } else {

            if (reply[4]==0) {
                printf("Speed M2= %d, forward  \n", counter);
                return  counter;
            } else {
                printf("Speed M2= %d, backward  \n", counter);
                return  counter;
            }

        }
    }

    //FIXME: Check that the previous login is ok
    return(0);

}



//Send duty cycle commands to the motor controller
void Signed_Duty_Motors(int dutycycle1, int dutycycle2)
{
    //The dutycycle variables are in the range +-1500
    unsigned char address;
    unsigned char command;

    address = g_roboclaw_address;

    command = 34; //Drive M1/M2 with signed duty cycle


    dutycycle1 = clamp_value(dutycycle1, 1500);
    dutycycle2 = clamp_value(dutycycle2, 1500);

    unsigned char *dc1_x;
    unsigned char *dc2_x;
    dc1_x = reinterpret_cast<unsigned char *>(&dutycycle1);
    dc2_x = reinterpret_cast<unsigned char *>(&dutycycle2);

    unsigned char cmd[7];
    cmd[0] = address;
    cmd[1] = command;
    cmd[2] = dc1_x[1];
    cmd[3] = dc1_x[0];
    cmd[4] = dc2_x[1];
    cmd[5] = dc2_x[0];
    cmd[6] = get_checksum(cmd, 6);


    device.write((const char*)&cmd, 7);

    printf("dc1= %x %x dc2 = %x %x\n", dc1_x[1], dc1_x[0], dc2_x[1], dc2_x[0]);

}

//FIXME: Check this function
void Signed_Speed_M(int motor,int vel)
{
    int Q;
    unsigned char address;
    unsigned char command;

    address = g_roboclaw_address;

    if (motor == 1) {
        command = 35;
        // Q=500;
    } else {
        command = 36;
    }


    Q=vel;

    //Q=71201;
    //Q=51201;
    //Q=51000;

    //Q=500;

    unsigned char *Qs_x;
    Qs_x = reinterpret_cast<unsigned char *>(&Q);

    unsigned char cmd[7];


    cmd[0] = address;
    cmd[1] = command;

    cmd[2] = Qs_x[3];
    cmd[3] = Qs_x[2];
    cmd[4] = Qs_x[1];
    cmd[5] = Qs_x[0];

    cmd[6] = get_checksum(cmd, 6);

    device.write((const char*)&cmd, 7);


}


//Main function to set the speed of the motors
void SignedSpeed_MIX(int QspeedM1,int QspeedM2)
{
    int Q;
    int Q2;
    unsigned char address;
    unsigned char command;

    address = g_roboclaw_address;
    command = 37;


    Q = QspeedM1;
    Q2 = QspeedM2;


    unsigned char *Qs_x;
    Qs_x = reinterpret_cast<unsigned char *>(&Q);

    unsigned char *Qs_x2;
    Qs_x2 = reinterpret_cast<unsigned char *>(&Q2);

    unsigned char cmd[11];

    cmd[0] = address;
    cmd[1] = command;

    cmd[2] = Qs_x[3];
    cmd[3] = Qs_x[2];
    cmd[4] = Qs_x[1];
    cmd[5] = Qs_x[0];

    cmd[6] = Qs_x2[3];
    cmd[7] = Qs_x2[2];
    cmd[8] = Qs_x2[1];
    cmd[9] = Qs_x2[0];

    cmd[10] = get_checksum(cmd, 10);

    device.write((const char*)&cmd, 11);
}




void set_QPID1(int motor)
{

    //M1 = Right motor
    //M2 = Left motor

    int Q, P, I, D;

    unsigned char address = g_roboclaw_address;
    unsigned char command;

    if (motor == 1)
    {
        command = 28;
    }
    else
    {
        command = 29;
    }

    Q = tortugabot_pps_max; //to get this value, get the motors to run with 100% duty cycle, and look at the counts per seconds
    P = tortugabot_pid_p;  //default: 65536
    I = tortugabot_pid_i; //          32768
    D = tortugabot_pid_d; //          16384

    unsigned char *Qs_x;
    Qs_x = reinterpret_cast<unsigned char *>(&Q);
    unsigned char *Ps_x;
    Ps_x = reinterpret_cast<unsigned char *>(&P);
    unsigned char *Is_x;
    Is_x = reinterpret_cast<unsigned char *>(&I);
    unsigned char *Ds_x;
    Ds_x = reinterpret_cast<unsigned char *>(&D);


    unsigned char cmd[19];
    cmd[0] = address;
    cmd[1] = command;

    cmd[2] = Ds_x[3];
    cmd[3] = Ds_x[2];
    cmd[4] = Ds_x[1];
    cmd[5] = Ds_x[0];


    cmd[6] = Ps_x[3];
    cmd[7] = Ps_x[2];
    cmd[8] = Ps_x[1];
    cmd[9] = Ps_x[0];

    cmd[10] = Is_x[3];
    cmd[11] = Is_x[2];
    cmd[12] = Is_x[1];
    cmd[13] = Is_x[0];

    cmd[14] = Qs_x[3];
    cmd[15] = Qs_x[2];
    cmd[16] = Qs_x[1];
    cmd[17] = Qs_x[0];

    cmd[18] = get_checksum(cmd, 18);

    device.write((const char*)&cmd, 19);

}








//Callback function for the ROS subscriber to the velocity command topic
//Should pet the watchdog
void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{

    last_cmd_received= ros::Time::now();


    const float D = tortugabot_wheel_diameter;
    const float wheel_circumference = PI * D;  // 2*pi*r = circumference
    const float L = tortugabot_wheel_to_wheel_distance;

    const float ppv = tortugabot_ticks_per_turn;

    const float ticks_per_meter = ppv / wheel_circumference;

    float VL = 0;
    float VR = 0;
    float VL_ticks = 0;
    float VR_ticks = 0;

    //const float pps_max = tortugabot_pps_max;// max encoder pulses with dutycycle=100%
    //float vel_lin_max = (pps_max / ppv) * wheel_circumference;  //max ticks/sec / ticks/turn = turns/s * circumference
    //float vel_ang_max = vel_lin_max * 2 / L;

    float vel_lin_esc;
    float vel_ang_esc;

    vel_lin_esc = cmd_vel->linear.x;
    vel_ang_esc = cmd_vel->angular.z;

    //ROS_INFO("request: vel_ang=%4.2f, , vel_lin=%4.2f ", (float) (vel_ang_esc), (float)(vel_lin_esc));

    //FIXME: Check this equations for the wheel speeds (is vel_ang_esc in radians?)
    //VR and VL in m/s
    VR = (2 * vel_lin_esc + vel_ang_esc * L) / 2;   // (2V+wL)/2=VR
    VL = 2 * vel_lin_esc - VR; //2v-VR=VL

    //VR_ticks and VL_ticks in ticks/s
    VR_ticks = left_motor_sign_ * VR * ticks_per_meter;
    VL_ticks = right_motor_sign_ * VL * ticks_per_meter;

    //printf("VR_t = %f  VL_t = %f \n", VR_ticks, VL_ticks);



    //FIXME: Clamp to maximum speeds decently
    SignedSpeed_MIX(VR_ticks, VL_ticks);
}


//The max qpps (quad ticks per second) is needed for tuning the PID to the specific motors
//This function runs at Duty-cycle=100% for 10s and prints the change in encoder ticks per second
void find_max_qpps_experiment(){

    reset_counts();

    Signed_Duty_Motors(1500, 1500);
    int old_enc1 = 0;
    int old_enc2 = 0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


    for (unsigned int i = 0; i < 10; ++i){
        sleep(1);
        current_time = ros::Time::now();
        int enc1 = Read_Quadrature_M(1);
        int enc2 = Read_Quadrature_M(2);
        printf("enc1 = %d\tenc2 = %d\n", enc1, enc2);
        double dt = (current_time - last_time).toSec();


        int qpps1 = (enc1 - old_enc1) / dt;
        int qpps2 = (enc2 - old_enc2) / dt;

        last_time = current_time;
        old_enc1 = enc1;
        old_enc2 = enc2;

        printf("qpps1 = %d  qpps2 = %d\n", qpps1, qpps2);

    }


    Signed_Duty_Motors(0,0);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboclaw_controller");
    ROS_INFO("RoboClaw controller coming up");

    ros::NodeHandle n("~");

    std::string serial_port_;
    int baudrate_, roboclaw_address_;
    bool invert_left_motor_sign_;
    bool invert_right_motor_sign_;

    double wheel_diameter, wheel_to_wheel_distance;
    int ticks_per_turn, pps_max, pid_p, pid_i, pid_d;

    std::string odom_frame_;
    std::string base_footprint_frame_;


    n.param<std::string>("serial_port", serial_port_, "/dev/ttyACM0");
    n.param<int>("baudrate", baudrate_, 1000000);
    n.param<int>("roboclaw_address", roboclaw_address_, 128);

    n.param<std::string>("odom_frame", odom_frame_, "odom");
    n.param<std::string>("base_footprint_frame", base_footprint_frame_, "base_footprint");


    n.param<bool>("invert_left_motor_sign", invert_left_motor_sign_, false);
    n.param<bool>("invert_right_motor_sign", invert_right_motor_sign_, false);

    if (invert_left_motor_sign_) {
      ROS_INFO("Inverting left motor sign");
      left_motor_sign_ = -1;
    }

    if (invert_right_motor_sign_) {
      ROS_INFO("Inverting right motor sign");
      right_motor_sign_ = -1;
    }

    n.param<double>("wheel_diameter", wheel_diameter, 0.125);
    n.param<double>("wheel_to_wheel_distance", wheel_to_wheel_distance, 0.357);
    n.param<int>("ticks_per_turn", ticks_per_turn, 980); //EMG49 -> 980  EMG30 -> 360
    n.param<int>("pps_max", pps_max, 2300);  //aximum number of ticks/s when running with duty cycle of 100%
    n.param<int>("pid_p", pid_p, 65000);
    n.param<int>("pid_i", pid_i, 32768);
    n.param<int>("pid_d", pid_d, 16384);

    //setting the global variables with the parmeter values
    g_roboclaw_address = roboclaw_address_;
    tortugabot_wheel_diameter = wheel_diameter;
    tortugabot_wheel_to_wheel_distance = wheel_to_wheel_distance;
    tortugabot_ticks_per_turn = ticks_per_turn;
    tortugabot_pps_max = pps_max;
    tortugabot_pid_p = pid_p;
    tortugabot_pid_i = pid_i;
    tortugabot_pid_d = pid_d;


    ROS_INFO("Configuration: wheel_diam=%5.3fm wheel_to_wheel_d=%5.3fm ticks_per_turn=%d", wheel_diameter, wheel_to_wheel_distance, ticks_per_turn);
    ROS_INFO("               pps_max=%d pid_p=%d pid_i=%d pid_d=%d", pps_max, pid_p, pid_i, pid_d);



    //We have all the info. Now let us start up

    //Get the serial port going
    int retry_counter = 0;
    int max_retries = 60;
    bool worked = false;

    while (retry_counter < max_retries) {
      try { 
        device.open(serial_port_.c_str(), baudrate_);
        retry_counter = max_retries;
        worked = true;
      }  // USB devices connect at 1Mbps
      catch(cereal::Exception& e)
      {
          retry_counter += 1;
          ROS_WARN("Could not open the serial port (%s). Will retry in 2s. Retry #%d/%d", serial_port_.c_str(), retry_counter, max_retries);
          ros::Duration(2.0).sleep();
      }

    }

    if (!worked) {
      ROS_FATAL("Giving up on opening the serial port. Exiting.");
      ROS_BREAK();
      return(-1);
    }

 
    ROS_INFO("The serial port (%s) is opened at %d bps.", serial_port_.c_str(), device.baudRate());

    //Starting fresh, just in case
    device.flush();

    ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 3, cmdVelReceived);


    //Initialize the roboclaw
    reset_counts();
    set_QPID1(1);
    set_QPID1(2);

    //FIXME: Check that the PID values got applied

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;


    int count_RL = 0;
    int count_RR = 0;

    double diameterWheel = tortugabot_wheel_diameter;
    int countsPerRevolution = tortugabot_ticks_per_turn;
    double mDistancePerCount = (PI * diameterWheel) / countsPerRevolution;

    double trackWidth = tortugabot_wheel_to_wheel_distance;

    double DistancePerCount = (PI * diameterWheel) / (double)countsPerRevolution;
    double mRadiansPerCount = DistancePerCount / trackWidth;
    double mX = 0;
    double mY = 0;
    double mHeading = 0;
    double mPreviousLeftCounts = 0;
    double mPreviousRightCounts = 0;
    double leftCounts;
    double rightCounts;


    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    reset_counts();


    //EXPERIMENTS
    //find_max_qpps_experiment();
    //SignedSpeed_MIX(360.0, 360.0);
    //sleep(10);
    //SignedSpeed_MIX(0.0, 0.0);

    //find_max_qpps_experiment();
    //return(0);


    ros::Rate r(100);
    while(ros::ok())
    {

        //    ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();

        count_RR = right_motor_sign_ * Read_Quadrature_M(1);
        count_RL = left_motor_sign_  * Read_Quadrature_M(2);

        rightCounts = count_RR;
        leftCounts = count_RL;

        float deltaLeft = leftCounts- mPreviousLeftCounts;
        float deltaRight = rightCounts - mPreviousRightCounts;
        float deltaDistance = 0.5f * (float)(deltaLeft + deltaRight) * mDistancePerCount;
        float deltaX = deltaDistance * (float)cos(mHeading);
        float deltaY = deltaDistance * (float)sin(mHeading);
        double deltaHeading = (deltaRight - deltaLeft) * mRadiansPerCount;

        //accumulate the change in position
        mX += deltaX;
        mY += deltaY;
        mHeading += deltaHeading;

        // limit heading to -Pi <= heading < Pi
        if (mHeading > PI)
            mHeading -= 2 * PI;
        else if (mHeading <= -PI)
            mHeading += 2 * PI;



        vx = deltaX / dt;
        vy = deltaY / dt;
        vth = deltaHeading / dt;


        mPreviousLeftCounts = leftCounts;
        mPreviousRightCounts = rightCounts;


        //since all odometry is 6DOF we'll need a quaternion created from yaw
        //    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(mHeading);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;

        odom_trans.header.frame_id = odom_frame_.c_str();
        odom_trans.child_frame_id = base_footprint_frame_.c_str();

        odom_trans.transform.translation.x = mX;
        odom_trans.transform.translation.y = mY; 
        odom_trans.transform.translation.z = 0.0; //robot stuck on the floor :)
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position (in the odom frame of reference)
        odom.pose.pose.position.x = mX;//x;
        odom.pose.pose.position.y = mY;//y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = base_footprint_frame_.c_str();
        //the twist in the odom message is in the reference frame of the child_frame_id
        odom.twist.twist.linear.x = deltaDistance / dt;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;


        //FIXME: Check this watchdog, it looks suspicious
        if ((last_cmd_received != ros::Time()) && (current_time > last_cmd_received+half_seconds))

        {
            SignedSpeed_MIX(0,0);
            //last_cmd_received = ros::Time();
        }

        //FIXME: Should check temperatures, voltages, etc, at lower rate and publish diagnostics

        ros::spinOnce();               // check for incoming messages
        r.sleep();


    }




}

