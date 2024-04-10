#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <craic_mission/Px4sp_msgs.h>
mavros_msgs::PositionTarget setpoint; // 位置速度控制消息类
mavros_msgs::State current_state;
// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;

float Px4Sp_Buf[5] = {0, 0, 0, 0, 0};

// 订阅时的回调函数，接受到该消息体的内容时执行里面的内容，内容是储存飞控当前的状态
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pos = *msg;
}

void MsgSp_Callback(const craic_mission::Px4sp_msgs::ConstPtr &msg)
{
    ROS_INFO("px4_commande topic receive");
    ROS_INFO("x:%f,y:%f,z:%f,yaw:%f", msg->x, msg->y, msg->z, msg->yaw);
    Px4Sp_Buf[0] = msg->x;
    Px4Sp_Buf[1] = msg->y;
    Px4Sp_Buf[2] = msg->z;
    Px4Sp_Buf[3] = msg->yaw;
    Px4Sp_Buf[4] = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Px4_ctl_node"); // ros初始化，最后一个参数为节点名称
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber command_sub = nh.subscribe("px4_command", 10, MsgSp_Callback);
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    // 官方要求local_pos_pub发布速率必须快于2Hz，这里设置为20Hz
    ros::Rate rate(20.0);
    // 等待飞控和MAVROS建立连接，current_state是我订阅的MAVROS的状态，在收到心跳包之后连接成功跳出循环
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // 这里开始收到数据 然后解锁启动
    while (Px4Sp_Buf[4] == 0)
    {
        ROS_INFO("WAITING FOR TAKEOFF");
        ros::spinOnce();
        rate.sleep();
    }

    setpoint.header.stamp = ros::Time::now();
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint.type_mask = // 使用位置控制
                         // mavros_msgs::PositionTarget::IGNORE_PX |
                         // mavros_msgs::PositionTarget::IGNORE_PY |
                         // mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        // mavros_msgs::PositionTarget::IGNORE_YAW;
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setpoint.position.x = 0;
    setpoint.position.y = 0;
    setpoint.position.z = 0;
    // 在进入Offboard模式之前，必须已经启动了local_pos_pub数据流，否则模式切换将被拒绝。
    // 这里的100可以被设置为任意数 100 少一点
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    // 建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"，作用便是用于后面的客户端与服务端之间的通信（服务）
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    // 设定无人机保护模式 POSTION
    mavros_msgs::SetMode offb_setPS_mode;
    offb_setPS_mode.request.custom_mode = "POSCTL";
    // 建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"，作用便是用于后面的客户端与服务端之间的通信（服务）
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    // 大循环，只要节点还在ros::ok()的值就为正
    int start_flag = 0;
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.1)) && start_flag == 0)
        {
            // 客户端set_mode_client向服务端offb_set_mode发起请求call，然后服务端回应response将模式返回，这就打开了offboard模式
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                // 切换到 OFFBOARD 模式
                ROS_INFO("OFFBOARD MODE");
            }
            last_request = ros::Time::now();
        }

        // 判断当前状态是否解锁，如果没有解锁，则进入if语句内部
        // 这里是5秒钟进行一次判断，避免飞控被大量的请求阻塞
        else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.1)) && start_flag == 0)
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("lift off!");
                start_flag = 1;
            }
            last_request = ros::Time::now();
        }
        else
        {
            setpoint.position.x = Px4Sp_Buf[0];
            setpoint.position.y = Px4Sp_Buf[1];
            setpoint.position.z = Px4Sp_Buf[2];
            setpoint.yaw = Px4Sp_Buf[3];
        }
        // 发布位置信息，所以综上飞机只有先打开offboard模式然后解锁才能飞起来
        setpoint_pub.publish(setpoint);
        // 当spinOnce函数被调用时，会调用回调函数队列中第一个回调函数，这里回调函数是state_cb函数
        ros::spinOnce();
        // 根据前面ros::Rate rate(20.0);制定的发送频率自动休眠 休眠时间 = 1/频率
        rate.sleep();
    }
    return 0;
}
