#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>


#include <sensor_msgs/msg/joy.hpp>
#include "common_interfaces/msg/vehicle_control.hpp"
#include "common_interfaces/msg/vehicle_mode.hpp"

#include "rclcpp/rclcpp.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;


class JoystickCommand : public rclcpp::Node
{
  public:

    JoystickCommand() : Node("joystick_command")
    {

      declare_parameters();
      load_parameters_and_initialization();  

      create_publishers();
      create_subscriptions();
      create_timers();      
      
    }
  
  private:

    // Topic name list
    std::string joy_topic_name; // = "/joy";
    std::string mode_topic_name; // = "/autonomous_driver";
    std::string cmd_topic_name;  // = "/hw/manual_cmd";
    
    int msg_max_period;
    int command_rate;
    int mode_rate;

    int throttle_axis;
    float throttle_scale;
    int steering_axis;
    float steering_scale;
    int neutral_button;
    int manual_button;
    int autonomous_button;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subs_joy;
    //rclcpp::Subscription<common_interfaces::msg::VehicleMode>::SharedPtr subs_mode;
    
    rclcpp::TimerBase::SharedPtr timer_subs_mode_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_joy_timeout;

    rclcpp::Publisher<common_interfaces::msg::VehicleControl>::SharedPtr pub_command;
    rclcpp::Publisher<common_interfaces::msg::VehicleMode>::SharedPtr pub_mode;
    
    std::atomic<bool> subs_mode_ok;
    std::atomic<bool> subs_joy_ok;

    std::atomic<int> mode;


    std::atomic<int> command_counter;
    std::atomic<int> mode_counter;


    void timer_subs_mode_timeout_callback()
    {
      //RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_mode");
      subs_mode_ok = false;
    }

    void timer_subs_joy_timeout_callback()
    {
      RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_joy");

    }

    void subs_mode_callback(const common_interfaces::msg::VehicleMode::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "subs_mode_callback");
      subs_mode_ok = true;
      mode = msg->mode;

      start_timer_subs_mode_timeout();
    }

    void subs_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "subs_joy_callback"); 

      float throttle = msg->axes[throttle_axis] * throttle_scale;
      float steering_angle = msg->axes[steering_axis] * steering_scale;

      // in manual mode the 'mode' must be mantained so i reset i every time
      if (mode = 1) {
        mode = 0;
      }

      if (msg->buttons[neutral_button]){
        mode = 0;
      } else if (msg->buttons[manual_button]) {
        mode = 1;
      } else if (msg->buttons[autonomous_button]) {
        mode = 2;
      }

      command_counter++;
      mode_counter++;

      // out command msg
      if (command_counter >= command_rate) {
        common_interfaces::msg::VehicleControl vehicle_control_msg;
        vehicle_control_msg.throttle = throttle;
        vehicle_control_msg.steering_angle = steering_angle;
        pub_command->publish(vehicle_control_msg);
        RCLCPP_INFO(this->get_logger(), "Command->\tThrottle:%f\tSteeringAngle:%f",
                                          vehicle_control_msg.throttle,
                                         vehicle_control_msg.steering_angle);
      }
      

      // out mode msg
      if (command_counter >= command_rate) {
        common_interfaces::msg::VehicleMode vehicle_mode_msg;
        vehicle_mode_msg.mode = mode;
        pub_mode->publish(vehicle_mode_msg);
        RCLCPP_INFO(this->get_logger(), "Mode->\t%d", vehicle_mode_msg.mode);
      }

      start_timer_subs_joy_timeout();
    }

    void create_timers() {

      start_timer_subs_mode_timeout();
      start_timer_subs_joy_timeout();

    }

    void start_timer_subs_mode_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
      timer_subs_mode_timeout = this->create_wall_timer(
        timeout_time, std::bind(&JoystickCommand::timer_subs_mode_timeout_callback, this)
        );
    }

    void start_timer_subs_joy_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
      timer_subs_joy_timeout = this->create_wall_timer(
        timeout_time, std::bind(&JoystickCommand::timer_subs_joy_timeout_callback, this)
        );
    }

    void create_subscriptions() {
      //subs_mode = this->create_subscription<common_interfaces::msg::VehicleMode>(
      //  mode_topic_name, 10, std::bind(&JoystickCommand::subs_mode_callback, this, _1));

      subs_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_name, 10, std::bind(&JoystickCommand::subs_joy_callback, this, _1));
    }

    void create_publishers() {
      pub_command = this->create_publisher<common_interfaces::msg::VehicleControl>(cmd_topic_name, 10);
      pub_mode = this->create_publisher<common_interfaces::msg::VehicleMode>(mode_topic_name, 10);
      
    }

    void load_parameters_and_initialization() {

      this->get_parameter("topics.in_joy", joy_topic_name);
      this->get_parameter("topics.in_mode", mode_topic_name);
      this->get_parameter("topics.out_cmd", cmd_topic_name);
      this->get_parameter("in_msg_max_period", msg_max_period);
      this->get_parameter("command_rate", command_rate);
      this->get_parameter("mode_rate", mode_rate);
      
      this->get_parameter("throttle_axis.index", throttle_axis);
      this->get_parameter("throttle_axis.scale", throttle_scale);
      this->get_parameter("steering_axis.index", steering_axis);
      this->get_parameter("steering_axis.scale", steering_scale);
      this->get_parameter("buttons.neutral_mode_index", neutral_button);
      this->get_parameter("buttons.manual_mode_index", manual_button);
      this->get_parameter("buttons.autonomous_mode_index", autonomous_button);
      
    }

    void declare_parameters() {
      this->declare_parameter<std::string>("topics.in_joy", "");
      this->declare_parameter<std::string>("topics.in_mode", "");
      this->declare_parameter<std::string>("topics.out_cmd", "");

      this->declare_parameter<int>("in_msg_max_period", 5000);
      this->declare_parameter<int>("command_rate", 1);
      this->declare_parameter<int>("mode_rate", 10);

      this->declare_parameter<int>("throttle_axis.index", 0);
      this->declare_parameter<float>("throttle_axis.scale", 1.0);
      this->declare_parameter<int>("steering_axis.index", 1);
      this->declare_parameter<float>("steering_axis.scale", 1.0);
      this->declare_parameter<int>("buttons.neutral_mode_index", 0);
      this->declare_parameter<int>("buttons.manual_mode_index", 1);
      this->declare_parameter<int>("buttons.autonomous_mode_index", 2);

     
    }      
    

    

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickCommand>());
    rclcpp::shutdown();
    
    return 0;
  }
