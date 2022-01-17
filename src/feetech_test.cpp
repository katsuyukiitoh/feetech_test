#include <memory>
#include <bits/stdc++.h>
#include <chrono>
#include <functional>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define ADDR_WORK_MODE        0x21 // 1Byte, EPROM
#define ADDR_TORQUE_ENABLE    0x28 // 1Byte, SRAM
#define ADDR_GOAL_ACCELERATE  0x29 // 1Byte, SRAM
#define ADDR_GOAL_VELOCITY    0x2E // 2Bytes, SRAM
#define ADDR_PRESENT_POSITION 0x38 // 2Bytes, SRAM
#define ADDR_PRESENT_VELOCITY 0x3A // 2Bytes, SRAM

#define TORQUE_ENABLE  1
#define TORQUE_DISABLE 0

#define VELOCITY_CONTROL_MODE 1

#define PROTOCOL_VERSION 1.0
  
class feetechTest : public rclcpp::Node
{
  
public:
  feetechTest()
  : Node("feetech_control")
  {
    /*
     * パラメータファイルから設定取得
     */
    port_name_ = "/dev/ttyUSB0";
    this->declare_parameter("port", port_name_);
    this->get_parameter("port", port_name_);
    
    baud_rate_ = 1000000;
    this->declare_parameter("baud_rate", baud_rate_);
    this->get_parameter("baud_rate", baud_rate_);

    wheel_radius_ = 0.033;
    this->declare_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("wheel_radius", wheel_radius_);

    wheel_separation_ = 0.100;
    this->declare_parameter("wheel_separation", wheel_separation_);
    this->get_parameter("wheel_separation", wheel_separation_);
    
    odom_frame_ = "odom";
    this->declare_parameter("odom_frame", odom_frame_);
    this->get_parameter("odom_frame", odom_frame_);

    odom_child_frame_ = "base_footprint";
    this->declare_parameter("odom_child_frame", odom_child_frame_);
    this->get_parameter("odom_child_frame", odom_child_frame_);

    id_L_ = 1;
    this->declare_parameter("id_l", id_L_);
    this->get_parameter("id_l", id_L_);
    
    id_R_ = 2;
    this->declare_parameter("id_r", id_R_);
    this->get_parameter("id_r", id_R_);
    
    joint_state_l_ = "wheel_left";
    this->declare_parameter("joint_state_l", joint_state_l_);
    this->get_parameter("joint_state_l", joint_state_l_);
    
    joint_state_r_ = "wheel_right";
    this->declare_parameter("joint_state_r", joint_state_r_);
    this->get_parameter("joint_state_r", joint_state_r_);

    resolution_ = 4096;
    this->declare_parameter("resolution", resolution_);
    this->get_parameter("resolution", resolution_);

    g_x_  = 0.0;
    g_y_  = 0.0;
    g_th_ = 0.0;

    g_last_time_ = this->now();

    usb2ttl_init();             // モータ制御I/F関連初期化

    model_check(this->id_L_);
    //    set_wheelMode(this->id_L_);
    set_torqueEnable(this->id_L_, TORQUE_ENABLE);
    set_goalAccel(this->id_L_, 100);
    
    model_check(this->id_R_);
    //    set_wheelMode(this->id_R_);
    set_torqueEnable(this->id_R_, TORQUE_ENABLE);
    set_goalAccel(this->id_R_, 100);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&feetechTest::topic_callback, this, _1));
    
    odom_publisher_= this->create_publisher<nav_msgs::msg::Odometry>(
      odom_frame_.c_str(), 10);
    jointstate_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
    
    timer_ = this->create_wall_timer(
      10ms, std::bind(&feetechTest::odometryPublish_callback, this));

    odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  }

  /*
   * デストラクタ
   * 終了時はトルクを抜く
   */  
  ~feetechTest()
  {
    set_torqueEnable(this->id_L_, TORQUE_DISABLE);
    set_torqueEnable(this->id_R_, TORQUE_DISABLE);
  }


private:
  /*
   * USB2TTLの初期化
   */
  int usb2ttl_init() {
    int comm_result;

    this->portHandler_ = dynamixel::PortHandler::getPortHandler(this->port_name_.c_str());
    this->packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    comm_result = portHandler_->openPort();
    if (comm_result == false)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
      return -1;
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to open the port(%s).",
        this->port_name_.c_str());
    }

    comm_result = portHandler_->setBaudRate(this->baud_rate_);
    if (comm_result == false)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
      return -1;
    } else {
      RCLCPP_INFO(this->get_logger(), "Succeeded to set baudrate(%d)",
        this->baud_rate_);
    }

    return 0;
  }

  /*
   * モータの存在確認
   */  
  int model_check(uint8_t id) {
    uint16_t model_number = 0;
    uint8_t err = 0;
    int comm_result;

    comm_result = packetHandler_->ping(
      portHandler_,
      id,
      &model_number,
      &err
      );
    
    if (comm_result == COMM_SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "id : %d, model : %d", id, model_number);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to ping(ID: %d), %d", id, comm_result);
      return -1;
    }
    
    return 0;
  }

  /*
   * モータをVelocityControlModeにする
   * ADDR_WORK_MODEはEPROM領域のためWindowsのFD.exeで変更したほうが良さげ
   */
  int set_wheelMode(uint8_t id)
  {
    uint8_t err = 0;
    int comm_result;

    comm_result = packetHandler_->write1ByteTxRx(
      portHandler_,
      id,
      ADDR_WORK_MODE,
      VELOCITY_CONTROL_MODE,
      &err);
    
    if (comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "id : %d, Failed to change wheel mode", id);
      return -1;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "id : %d, Succeed to change wheel mode", id);
    }
    
    return 0;
  }

  /*
   * モータをトルクがかけられるようにする
   */
  int set_torqueEnable(uint8_t id, uint8_t enable)
  {
    uint8_t err = 0;
    int comm_result;

    comm_result = packetHandler_->write1ByteTxRx(
      portHandler_,
      id,
      ADDR_TORQUE_ENABLE,
      enable,
      &err);
    if (comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
        "id : %d, Failed to change torque change(%d)", id, enable);
      return -1;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
        "id : %d, Succeed to change torque change(%d)", id, enable);
    }
    
    return 0;
  }

  /*
   * 加速度設定
   */
  int set_goalAccel(uint8_t id, uint8_t acc)
  {
    uint8_t err = 0;
    int comm_result;

    comm_result = packetHandler_->write1ByteTxRx(
      portHandler_,
      id,
      ADDR_GOAL_ACCELERATE,
      acc,
      &err);
    
    if (comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "id : %d, Failed to change goal Accel to %d",
        id, acc);
      return -1;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "id : %d, Succeed to change goal Accel to %d",
        id, acc);
    }
    
    return 0;
  }
  
  /*
   * cmd_velを受けたときのコールバック関数
   * 並進・回転速度をもらって左右のモータを回す
   */
  int topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v = msg->linear.x;       // 移動ロボットの並進速度[m/s]
    double w = msg->angular.z;      // 移動ロボットの回転角速度[rad/s]
    double wR, wL;                  // ホイールの回転角速度[rad/s]
    double val_R, val_L;            // ホイールの一秒あたりの回転ステップ数[step/s]

    wR = v / wheel_radius_ + wheel_separation_ * w / 2.0 / wheel_radius_;
    wL = v / wheel_radius_ - wheel_separation_ * w / 2.0 / wheel_radius_;
    
    val_R = ( 1.0) * resolution_ * wR / 2.0 / M_PI;
    val_L = (-1.0) * resolution_ * wL / 2.0 / M_PI; // 回転方向が右側とは逆のため-1を掛けている

    //RCLCPP_INFO(this->get_logger(), "Goal Velocity (%f, %f)", val_L, val_R);    

    setGoalVelocity(id_R_, (int16_t)val_R);
    setGoalVelocity(id_L_, (int16_t)val_L);

    return 0;
  }

  /*
   * モータに目標速度を与える
   * goalVelはstep/s
   */  
  int setGoalVelocity(uint8_t id, int16_t goalVel)
  {
    uint8_t err = 0;
    int comm_result;
    uint16_t val;

    // 負の値を設定するときは15ビット目を1にする（Feetech仕様)
    val = abs(goalVel);
    if (goalVel < 0) {
      val |= 1 << 15;
    }

    comm_result = packetHandler_->write2ByteTxRx(
      portHandler_,
      id,
      ADDR_GOAL_VELOCITY,
      val,
      &err
      );
    if (comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getTxRxResult(comm_result));
      return -1;
    }
    else if (err != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getRxPacketError(err));
      return -1;
    }
    
    return 0;
  }

  /*
   * オドメトリ・JointState・TFをパブリッシュするコールバック関数
   */
  int odometryPublish_callback()
  {
    auto odom_msg = nav_msgs::msg::Odometry();
    auto state_msg = sensor_msgs::msg::JointState();
    
    int16_t stepPerSec_L, stepPerSec_R;
    int16_t prePos_L, prePos_R;
    double val_L, val_R;
    double rad_L, rad_R;
    
    rclcpp::Time current_time = this->now();

    // 現在速度(１秒あたりのステップ数)取得
    getPresentVelocity(id_L_, &stepPerSec_L);
    getPresentVelocity(id_R_, &stepPerSec_R);
    // 現在速度を[rad/s]に変換
    val_R = (int)stepPerSec_R * 2 * M_PI / resolution_ * ( 1.0);
    val_L = (int)stepPerSec_L * 2 * M_PI / resolution_ * (-1.0); // 回転方向が右とは逆のため-1を掛けている
    //RCLCPP_INFO(this->get_logger(), "Present Velocity (%f, %f)", val_L, val_R);

    // 現在位置取得
    getPresentPosition(id_L_, &prePos_L);
    getPresentPosition(id_R_, &prePos_R);
    //RCLCPP_INFO(this->get_logger(), "Present Position (%d, %d)", prePos_L, prePos_R);

    rad_R = prePos_R * 2 * M_PI / 4096;
    rad_L = 2 * M_PI - prePos_L * 2 * M_PI / 4096; // 回転方向が逆のため(2 * M_PI)から引く
    //RCLCPP_INFO(this->get_logger(), "Present Position (%f, %f)", rad_L, rad_R);
    
    double vl = wheel_radius_ * val_L; // [m/s]
    double vr = wheel_radius_ * val_R; // [m/s]
    
    double vx  = (vr + vl) / 2.0;              // [m/s]
    double vy  = 0.0;
    double vth = (vr - vl) / wheel_separation_; // [rad/s]
    
    rclcpp::Duration duration(current_time.nanoseconds() - g_last_time_.nanoseconds());
    double dt = duration.seconds();
    
    double delta_x = (vx * cos(g_th_) - vy * sin(g_th_)) * dt;
    double delta_y = (vx * sin(g_th_) + vy * cos(g_th_)) * dt;
    double delta_th = vth * dt;
    
    g_x_  += delta_x;
    g_y_  += delta_y;
    g_th_ += delta_th;

    // Odomを作ってパブリッシュする
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_.c_str();
    
    odom_msg.pose.pose.position.x = g_x_;
    odom_msg.pose.pose.position.y = g_y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, g_th_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.child_frame_id = odom_child_frame_.c_str();
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = vth;
    
    odom_publisher_->publish(odom_msg);

    // JointStatesを作ってパブリッシュする
    state_msg.header.stamp = current_time;
    state_msg.name.resize(2);
    state_msg.name[0] = joint_state_l_.c_str();
    state_msg.name[1] = joint_state_r_.c_str();
    state_msg.position.resize(2);
    state_msg.position[0] = rad_L;
    state_msg.position[1] = rad_R;
    
    jointstate_publisher_->publish(state_msg);
    
    // odomフレーム(TF)をブロードキャストする
    geometry_msgs::msg::TransformStamped odom_tf;

    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation      = odom_msg.pose.pose.orientation;

    odom_tf.header.frame_id = odom_msg.header.frame_id;
    odom_tf.child_frame_id  = odom_msg.child_frame_id;
    odom_tf.header.stamp    = odom_msg.header.stamp;

    odom_tf_broadcaster_->sendTransform(odom_tf);

    g_last_time_ = current_time;

    return 0;
  }

  /*
   * Dynamixelモータから回転速度を取得する
   */
  int getPresentVelocity(uint8_t id, int16_t *velocity)
  {
    uint8_t err = 0;
    int comm_result;

    comm_result = packetHandler_->read2ByteTxRx(
      portHandler_,
      id,
      ADDR_PRESENT_VELOCITY,
      reinterpret_cast<uint16_t *>(velocity),
      &err
      );

    if (comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getTxRxResult(comm_result));
      return -1;
    }
    else if (err != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getRxPacketError(err));
      return -1;
    }

    // 15bit目が0 ... 正の値
    // 15bit目が1 ... 負の値
    int pn_L = (*velocity >> 15) & 1;
    *velocity &= ~(1 << 15);
    if (pn_L == 1) {
      *velocity *= (-1);
    }

    return 0;
  }

  /*
   * Dynamixelモータから位置を取得する
   */
  int getPresentPosition(uint8_t id, int16_t *position)
  {
    uint8_t err = 0;
    int comm_result;

    comm_result = packetHandler_->read2ByteTxRx(
      portHandler_,
      id,
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint16_t *>(position),
      &err
      );

    if (comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getTxRxResult(comm_result));
      return -1;
    }
    else if (err != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler_->getRxPacketError(err));
      return -1;
    }
    
    return 0;
  }

  std::string port_name_;
  int baud_rate_;

  double wheel_radius_;
  double wheel_separation_;

  std::string odom_frame_;
  std::string odom_child_frame_;

  uint8_t id_L_;
  uint8_t id_R_;

  std::string joint_state_l_;
  std::string joint_state_r_;

  int resolution_;

  double g_x_;
  double g_y_;
  double g_th_;
  
  rclcpp::Time g_last_time_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

  dynamixel::PortHandler * portHandler_;
  dynamixel::PacketHandler * packetHandler_;
};

int main(int argc,  char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<feetechTest>());
  rclcpp::shutdown();

  return 0;
}

