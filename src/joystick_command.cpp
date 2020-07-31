#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

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
    std::string mode_topic_name; // = "/autonomous_driver";
    std::string cmd_topic_name;  // = "/hw/manual_cmd";
    
    int msg_max_period;
    int command_period;
    int mode_period;

    std::string joystick_device_path;
    bool joystick_test_print;
    int throttle_axis;
    bool throttle_reverse;
    float throttle_dead_range;
    int steering_axis;
    bool steering_reverse;
    float steering_dead_range;
    int neutral_button;
    int manual_button;
    int autonomous_button;

    
    //rclcpp::Subscription<common_interfaces::msg::VehicleMode>::SharedPtr subs_mode;
    
    rclcpp::TimerBase::SharedPtr timer_subs_mode_timeout;
    rclcpp::TimerBase::SharedPtr timer_command;
    rclcpp::TimerBase::SharedPtr timer_mode;

    rclcpp::Publisher<common_interfaces::msg::VehicleControl>::SharedPtr pub_command;
    rclcpp::Publisher<common_interfaces::msg::VehicleMode>::SharedPtr pub_mode;
    
    std::atomic<bool> subs_mode_ok;

    std::atomic<int> mode;
    std::atomic<float> throttle;
    std::atomic<float> steering_angle; 


    int joy_fd = -1;
    int num_of_axis = 0;
    int num_of_buttons = 0;
    char name_of_joystick[80];
    std::vector<char> joy_button;
    std::vector<int> joy_axis;


    void timer_subs_mode_timeout_callback()
    {
      //RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_mode");
      subs_mode_ok = false;
    }

    void subs_mode_callback(const common_interfaces::msg::VehicleMode::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "subs_mode_callback");
      subs_mode_ok = true;
      mode = msg->mode;

      start_timer_subs_mode_timeout();
    }

    void timer_command_callback()
    {
      
      read_joystick();
      if (joystick_test_print) {
        print_joystick();
      }      
      
      // out msg
      common_interfaces::msg::VehicleControl vehicle_control_msg;
      vehicle_control_msg.throttle = throttle;
      vehicle_control_msg.steering_angle = steering_angle;
      pub_command->publish(vehicle_control_msg);
      RCLCPP_INFO(this->get_logger(), "Command->\tThrottle:%f\tSteeringAngle:%f",
                                        vehicle_control_msg.throttle,
                                        vehicle_control_msg.steering_angle);
      
      


    }

    void timer_mode_callback()
    {
      
      // out msg
      common_interfaces::msg::VehicleMode vehicle_mode_msg;
      vehicle_mode_msg.mode = mode;
      pub_mode->publish(vehicle_mode_msg);
      RCLCPP_INFO(this->get_logger(), "Mode->\t%d", vehicle_mode_msg.mode);
      
      


    }

    void create_timers() {

      start_timer_subs_mode_timeout();

      start_timer_command();
      start_timer_mode();
    }

    void start_timer_subs_mode_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
      timer_subs_mode_timeout = this->create_wall_timer(
        timeout_time, std::bind(&JoystickCommand::timer_subs_mode_timeout_callback, this)
        );
    }
    
    void start_timer_command() {
      std::chrono::duration<int, std::milli> timeout_time(command_period);
      timer_command = this->create_wall_timer(
        timeout_time, std::bind(&JoystickCommand::timer_command_callback, this)
        );
    }

    void start_timer_mode() {
      std::chrono::duration<int, std::milli> timeout_time(mode_period);
      timer_mode = this->create_wall_timer(
        timeout_time, std::bind(&JoystickCommand::timer_mode_callback, this)
        );
    }

    void create_subscriptions() {
      //subs_mode = this->create_subscription<common_interfaces::msg::VehicleMode>(
      //  mode_topic_name, 10, std::bind(&JoystickCommand::subs_mode_callback, this, _1));
    }

    void create_publishers() {
      pub_command = this->create_publisher<common_interfaces::msg::VehicleControl>(cmd_topic_name, 10);
      pub_mode = this->create_publisher<common_interfaces::msg::VehicleMode>(mode_topic_name, 10);
      
    }

    void load_parameters_and_initialization() {

      this->get_parameter("topics.in_mode", mode_topic_name);
      this->get_parameter("topics.out_cmd", cmd_topic_name);
      this->get_parameter("in_msg_max_period", msg_max_period);
      this->get_parameter("command_period", command_period);
      this->get_parameter("mode_period", mode_period);

      this->get_parameter("joystick_device_path", joystick_device_path);
      this->get_parameter("joystick_test_print", joystick_test_print);
      
      this->get_parameter("throttle_axis.index", throttle_axis);
      this->get_parameter("throttle_axis.reverse", throttle_reverse);
      this->get_parameter("throttle_axis.dead_range", throttle_dead_range);
      this->get_parameter("steering_axis.index", steering_axis);
      this->get_parameter("steering_axis.reverse", steering_reverse);
      this->get_parameter("steering_axis.dead_range", steering_dead_range);
      this->get_parameter("buttons.neutral_mode_index", neutral_button);
      this->get_parameter("buttons.manual_mode_index", manual_button);
      this->get_parameter("buttons.autonomous_mode_index", autonomous_button);
      
      if (!initialize_joystick()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s",joystick_device_path);
      }
    }

    void declare_parameters() {
      this->declare_parameter<std::string>("topics.in_mode", "");
      this->declare_parameter<std::string>("topics.out_cmd", "");

      this->declare_parameter<int>("in_msg_max_period", 5000);
      this->declare_parameter<int>("command_period", 100);
      this->declare_parameter<int>("mode_period", 1000);

      this->declare_parameter<std::string>("joystick_device_path", "");
      this->declare_parameter<bool>("joystick_test_print", false);      
      this->declare_parameter<int>("throttle_axis.index", 0);
      this->declare_parameter<bool>("throttle_axis.reverse", false);
      this->declare_parameter<float>("throttle_axis.dead_range", 0.0);
      this->declare_parameter<int>("steering_axis.index", 1);
      this->declare_parameter<bool>("steering_axis.reverse", false);
      this->declare_parameter<float>("steering_axis.dead_range", 0.0);
      this->declare_parameter<int>("buttons.neutral_mode_index", 0);
      this->declare_parameter<int>("buttons.manual_mode_index", 1);
      this->declare_parameter<int>("buttons.autonomous_mode_index", 2);

     
    }   
    
    bool initialize_joystick() {

      if((joy_fd=open(joystick_device_path.c_str(),O_RDONLY)) < 0)
      {
        return false;
      }

      ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
      ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
      ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

      joy_button.resize(num_of_buttons,0);
      joy_axis.resize(num_of_axis,0);

      std::cout<<"Joystick: "<<name_of_joystick<<std::endl
        <<"  axis: "<<num_of_axis<<std::endl
        <<"  buttons: "<<num_of_buttons<<std::endl;

      fcntl(joy_fd, F_SETFL, O_NONBLOCK);   // using non-blocking mode

      //close(joy_fd);

      return true;
    }

    void read_joystick() {

      js_event js;

      while(read(joy_fd, &js, sizeof(js_event)) >= 0)
      {
        switch (js.type & ~JS_EVENT_INIT)
        {
        case JS_EVENT_AXIS:
          if((int)js.number>=joy_axis.size())  {std::cerr<<"err:"<<(int)js.number<<std::endl; continue;}
          joy_axis[(int)js.number]= js.value;
          break;
        case JS_EVENT_BUTTON:
          if((int)js.number>=joy_button.size())  {std::cerr<<"err:"<<(int)js.number<<std::endl; continue;}
          joy_button[(int)js.number]= js.value;
          break;
        }
        //usleep(100);
      }
      
      if (!throttle_reverse) {
        throttle = (float)joy_axis[throttle_axis]/32767.0;
      } else {
        throttle = -(float)joy_axis[throttle_axis]/32767.0;
      }
      if (throttle < throttle_dead_range && throttle > -throttle_dead_range) {
        throttle = 0;
      }


      if (!steering_reverse) {
        steering_angle = (float)joy_axis[steering_axis]/32767.0;
      } else {
        steering_angle = -(float)joy_axis[steering_axis]/32767.0;
      }
      if (steering_angle < steering_dead_range && steering_angle > -steering_dead_range) {
        steering_angle = 0;
      }

      if (joy_button[neutral_button]){
        mode = 0;
      } else if (joy_button[manual_button]) {
        mode = 1;
      } else if (joy_button[autonomous_button]) {
        mode = 2;
      } else {
        mode = 0;
      }      
      
    }

    void print_joystick() {

      std::cout<<"axis/10000: ";
      for(size_t i(0);i<joy_axis.size();++i)
        std::cout<<" "<<joy_axis[i]/1000;
      std::cout<<std::endl;

      std::cout<<"  button: ";
      for(size_t i(0);i<joy_button.size();++i)
        std::cout<<" "<<(int)joy_button[i];
      std::cout<<std::endl;

    }

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickCommand>());
    rclcpp::shutdown();
    
    return 0;
  }
