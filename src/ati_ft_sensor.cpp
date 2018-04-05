#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <functional>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <functional>
#include <realtime_tools/realtime_publisher.h>
#include <mutex>
#include <thread>

#include "sensoray826.h"

using namespace std;

const double SAMPLE_RATE = 1000; // Hz

enum SLOT_TIME {NONE = 0, DEFAULT = 50};

class atiFTSensorROS : public sensoray826_dev
{
    // SLOTs
    static const SLOTATTR slotAttrs[16];

    enum AD_INDEX {LEFT_FOOT = 0, RIGHT_FOOT = 8};

    // FT Datas

    const static double calibrationMatrixLFoot[6][6];
    const static double calibrationMatrixRFoot[6][6];

    double leftFootAxisData[6];
    double rightFootAxisData[6];
    double leftFootAxisData_prev[6];
    double rightFootAxisData_prev[6];

    double leftFootBias[6];
    double rightFootBias[6];

    bool isCalibration;
    double dCalibrationTime;

    double _calibLFTData[6];
    double _calibRFTData[6];
    int _calibTimeIndex;
    int _calibMaxIndex;

    //extencoder
    const static string JOINT_NAME[32];
    enum { max_length = 1024 };
    uint8_t data[max_length];
    std::mutex data_mutex;
    std::vector<double> q;
    std::vector<double> qdot;
    std::vector<double> leg_q;
    int sync = 0;

    const int LEFT_LEG_START_NUM = 0;
    const int RIGHT_LEG_START_NUM = 6;
    const int MAX_JOINT = 32;

    const double CNT_PER_REV = 983040.0;
    const double CNT_PER_DEG = CNT_PER_REV / 360.0;
    const double CNT_PER_RAD = CNT_PER_REV / ( M_PI * 2 );
    const double RAD_PER_CNT = ( M_PI * 2 ) / CNT_PER_REV;

    // ROS
    ros::Rate rate;
    ros::Publisher leftFootFTPublisher;
    ros::Publisher rightFootFTPublisher;
    ros::Publisher ftRawPublisher;
    ros::Publisher joint_pub;
    ros::Subscriber calibSubscriber;
    geometry_msgs::WrenchStamped leftFootMsg;
    geometry_msgs::WrenchStamped rightFootMsg;
    std_msgs::Float32MultiArray rawMsg;
    sensor_msgs::JointState joint_msg;

    int stampCount;


    void initCalibration()
    {
        for(int i=0; i<6; i++)
        {
            _calibLFTData[i] = 0.0;
            _calibRFTData[i] = 0.0;
        }
        _calibTimeIndex = 0;
        _calibMaxIndex = dCalibrationTime * SAMPLE_RATE;
        ROS_INFO("FT sensor calibration start... time = %.1lf sec, total %d samples ", dCalibrationTime, _calibMaxIndex);
    }

    void computeFTData()
    {
        if(isCalibration)
        {
            if(_calibTimeIndex < _calibMaxIndex)
            {
                for(int i=0; i<6; i++)
                {
                    double _lf = 0.0;
                    double _rf = 0.0;
                    for(int j=0; j<6; j++)
                    {
                        _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                        _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
                    }
                    _calibLFTData[i] += _lf / _calibMaxIndex;
                    _calibRFTData[i] += _rf / _calibMaxIndex;
                }

                _calibTimeIndex++;
            }
            else
            {
                isCalibration = false;
                for(int i=0; i<6; i++)
                {
                    leftFootBias[i] = _calibLFTData[i];
                    rightFootBias[i] = _calibRFTData[i];
                }

                ROS_INFO("LFT bias data = %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf",
                         leftFootBias[0], leftFootBias[1], leftFootBias[2], leftFootBias[3], leftFootBias[4], leftFootBias[5]);
                ROS_INFO("RFT bias data = %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf",
                         rightFootBias[0], rightFootBias[1], rightFootBias[2], rightFootBias[3], rightFootBias[4], rightFootBias[5]);

            }
//ROS_INFO("%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ", adcVoltages[0], adcVoltages[1], adcVoltages[2], adcVoltages[3], adcVoltages[4], adcVoltages[5]);
        }
        else
        {
            for(int i=0; i<6; i++)
            {
                double _lf = 0.0;
                double _rf = 0.0;
                for(int j=0; j<6; j++)
                {
                    _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                    _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
                }

                _lf -= leftFootBias[i];
                _rf -= rightFootBias[i];

                leftFootAxisData[i] = lowPassFilter(_lf, leftFootAxisData_prev[i], 1.0 / SAMPLE_RATE, 0.05);
                rightFootAxisData[i] = lowPassFilter(_rf, rightFootAxisData_prev[i], 1.0/ SAMPLE_RATE,0.05);
                leftFootAxisData_prev[i] = leftFootAxisData[i];
                rightFootAxisData_prev[i] = rightFootAxisData[i];
            }
        //ROS_INFO("%.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf ", adcVoltages[0], adcVoltages[1], adcVoltages[2], adcVoltages[3], adcVoltages[4], adcVoltages[5],adcVoltages[8], adcVoltages[9], adcVoltages[10], adcVoltages[11], adcVoltages[12], adcVoltages[13]);
        }
    }
    void publishFTData()
    {
        leftFootMsg.wrench.force.x = leftFootAxisData[0];
        leftFootMsg.wrench.force.y = leftFootAxisData[1];
        leftFootMsg.wrench.force.z = leftFootAxisData[2];
        leftFootMsg.wrench.torque.x = leftFootAxisData[3];
        leftFootMsg.wrench.torque.y = leftFootAxisData[4];
        leftFootMsg.wrench.torque.z = leftFootAxisData[5];
	//ROS_INFO("aaaaaaaaaaaaaaaa");//
        rightFootMsg.wrench.force.x = rightFootAxisData[0];
        rightFootMsg.wrench.force.y = rightFootAxisData[1];
        rightFootMsg.wrench.force.z = rightFootAxisData[2];
        rightFootMsg.wrench.torque.x = rightFootAxisData[3];
        rightFootMsg.wrench.torque.y = rightFootAxisData[4];
        rightFootMsg.wrench.torque.z = rightFootAxisData[5];

        leftFootMsg.header.stamp = ros::Time::now();
        rightFootMsg.header.stamp = leftFootMsg.header.stamp;

        rawMsg.data.resize(16);
        for(int i=0; i<16;i++)
            rawMsg.data[i] = adcVoltages[i];

        leftFootFTPublisher.publish(leftFootMsg);
        rightFootFTPublisher.publish(rightFootMsg);
        ftRawPublisher.publish(rawMsg);

        stampCount++;
    }

    void calibCallback(const std_msgs::Float32ConstPtr msg)
    {
        dCalibrationTime = msg->data;
        initCalibration();
        isCalibration = true;
        //calibration(msg->data);
    }
    void joint_publish()
    {
      data_mutex.lock();
      joint_msg.position = leg_q;
     // joint_msg.velocity = qdot;
      data_mutex.unlock();

      joint_pub.publish(joint_msg);
    }
public:
    atiFTSensorROS(ros::NodeHandle &nh) : sensoray826_dev(), isCalibration(false), stampCount(0), rate(SAMPLE_RATE), q(12), qdot(12), leg_q(12)
    {
        // ROS publish
        leftFootFTPublisher = nh.advertise<geometry_msgs::WrenchStamped>("ati_ft_sensor/left_foot_ft",5);
        rightFootFTPublisher = nh.advertise<geometry_msgs::WrenchStamped>("ati_ft_sensor/right_foot_ft",5);
        ftRawPublisher = nh.advertise<std_msgs::Float32MultiArray>("ati_ft_sensor/raw",5);
        calibSubscriber = nh.subscribe("ati_ft_sensor/calibration", 1, &atiFTSensorROS::calibCallback, this);
        joint_pub = nh.advertise<sensor_msgs::JointState>("/dyros_jet/ext_encoder",5);

        leftFootMsg.header.frame_id = "LeftFootFT";
        rightFootMsg.header.frame_id = "RightFootFT";

        joint_msg.name.resize(12);
        joint_msg.position.resize(12);
        joint_msg.velocity.resize(12);

        for(int i=0; i<12; i++)
        {
          joint_msg.name[i] = JOINT_NAME[LEFT_LEG_START_NUM + i];
        }

        // daq open
        sensoray826_dev::open();
        sensoray826_dev::analogSingleSamplePrepare(slotAttrs, 16);
    }

    void loop()
    {
        analogOversample();
        computeFTData();
        publishFTData();
        if (sync % 5 == 0)
        {
          multipleEncoder(leg_q, RAD_PER_CNT);
          joint_publish();
          sync = sync + 1;
        }
        else
        {
         sync = sync + 1;
        }
    }

};


const SLOTATTR atiFTSensorROS::slotAttrs[16] = {
    {0, DEFAULT}, {1, DEFAULT}, {2, DEFAULT}, {3, DEFAULT},
    {4, DEFAULT}, {5, DEFAULT}, {6, DEFAULT}, {7, NONE},
    {8, DEFAULT}, {9, DEFAULT}, {10, DEFAULT}, {11, DEFAULT},
    {12, DEFAULT}, {13, DEFAULT}, {14, DEFAULT}, {15, NONE}
};

const double atiFTSensorROS::calibrationMatrixLFoot[6][6] = {
    {-0.74957,      0.24281,    -23.71506,  466.60093,  18.22998,   -468.17685},
    { 7.19002,      -546.10473, -6.38277,   266.54205,  -11.70446,  277.80902},
    {-695.34548,    -16.86802,  -637.23056, 11.25426,   -777.28613, 38.67756},
    {-0.77968,      -7.75893,   14.91040,   3.51086,    -16.01169,  4.68867},
    {-17.44361,     -0.36882,   8.83623,    -6.62325,    9.17076,   6.19479},
    {-0.31783,      7.67933,    -0.32218,   7.62097,    -0.22801,   7.95418}
};
const double atiFTSensorROS::calibrationMatrixRFoot[6][6] = {
    {3.66373, -2.53887,-56.93166, 472.72786, 55.48633,-480.00204},
    { 69.76364,-550.39726,-18.98287, 268.38284,-51.71793, 281.63366},
    {-694.83314,-48.95910,-677.24401,-53.91778,-721.44018,-44.23236},
    {0.81357, -7.81489, 14.99121,  5.16497,-16.16116,  2.94277},
    {-17.34220, -1.31783,  9.88078, -5.83100,  7.85218,  7.37678},
    {-0.78857,  7.80856, -1.07929,  7.63313, -0.74397,  7.78721}
};

//Extencoder
const string atiFTSensorROS::JOINT_NAME[32] = { "L_HipYaw","L_HipRoll","L_HipPitch","L_KneePitch","L_AnklePitch","L_AnkleRoll",
                                "R_HipYaw","R_HipRoll","R_HipPitch","R_KneePitch","R_AnklePitch","R_AnkleRoll",
                                "WaistPitch","WaistYaw",
                                "L_ShoulderPitch","L_ShoulderRoll","L_ShoulderYaw","L_ElbowRoll","L_WristYaw","L_WristRoll","L_HandYaw",
                                "R_ShoulderPitch","R_ShoulderRoll","R_ShoulderYaw","R_ElbowRoll","R_WristYaw","R_WristRoll","R_HandYaw",
                                "HeadYaw", "HeadPitch", "R_Gripper", "L_Gripper"};
int main(int argc, char** argv) {

    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle nh;

    atiFTSensorROS ft(nh);

    ros::Rate r(SAMPLE_RATE);
    ROS_INFO("DAQ Initialize ...");
    for(int i=0; i<SAMPLE_RATE; i++)
        r.sleep();
    
    ROS_INFO("DAQ Initialize Done. Streaming started.");
    while(ros::ok())
    {
        ft.loop();
        r.sleep();

        ros::spinOnce();
    }
}
