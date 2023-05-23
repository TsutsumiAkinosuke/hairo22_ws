#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "custom_msgs/msg/commands.hpp"

using namespace std::chrono_literals;

class ConvertJoy2cmd : public rclcpp::Node
{
    public:

        ConvertJoy2cmd()
        : Node("convert_joy2cmd")
        {
            timer_ = this->create_wall_timer(50ms, std::bind(&ConvertJoy2cmd::timer_callback, this));
            publisher_ = this->create_publisher<custom_msgs::msg::Commands>("/ns/cmd", 10);
            subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&ConvertJoy2cmd::joy_callback, this, std::placeholders::_1));
            for(int i = 0; i < 4; i++){
                pre_buttons[i] = 0;
                operation_mode[i] = 0;
            }
            target_height_front = 0;
            target_height_back = 0;
        }
    
    private:

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

        custom_msgs::msg::Commands msg_cmd;

        int16_t pre_buttons[3];
        int16_t operation_mode[3];

        int16_t lr_button;
        int16_t pre_lr_button;
        int16_t target_height_front;
        int16_t target_height_back;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            for(int i = 0; i < 3; i++){
                if(msg->buttons[i] == 1 && pre_buttons[i] == 0){
                    operation_mode[i] = 1 - operation_mode[i];
                }
                pre_buttons[i] = msg->buttons[i];
            }

            // mecanum & crawler
            msg_cmd.mode[0] = operation_mode[0];
            if(operation_mode[0] == 0){
                msg_cmd.data[0]  = (int16_t)(msg->axes[1] * 32767);  // Joystick left  X
                msg_cmd.data[1]  = (int16_t)(msg->axes[0] * 32767);  // Joystick left  Y
                msg_cmd.data[2]   = (int16_t)(msg->axes[3] * 32767);  // Joystick pwm[2] X
            }else{
                msg_cmd.data[0]  = (int16_t)(msg->axes[1] * 32767);  // Joystick lest  Y
                msg_cmd.data[2]   = (int16_t)(msg->axes[4] * 32767);  // Joystick pwm[2] X
            }

            // decomission
            msg_cmd.mode[1] = operation_mode[1];
            msg_cmd.data[3] = (int16_t)((msg->buttons[4] - msg->buttons[5]) * 32767);   // Joystick L1 R1

            // rise & fall
            msg_cmd.mode[2] = operation_mode[2];
            msg_cmd.data[4] = (int16_t)(msg->axes[7] * 32767); // Joystick UP DOWN

            // electric suspention target height
            pre_lr_button = lr_button;
            lr_button = msg->axes[6];   // Joystick RIGHT LEFT
            if(lr_button != pre_lr_button && pre_lr_button == 0){
                if(msg->buttons[7] == 1){ // Joystick R2
                    target_height_front -= (lr_button*(5+msg->buttons[9]*5));
                }else if(msg->buttons[6] == 1){ //Joystick L2
                    target_height_back -= (lr_button*(5+msg->buttons[9]*5));
                }else{
                    target_height_front -= (lr_button*(5+msg->buttons[9]*5));
                    target_height_back -= (lr_button*(5+msg->buttons[9]*5));
                }
            }
            msg_cmd.data[5] = target_height_front;    // Joystick LR
            msg_cmd.data[6] = target_height_back;

            // cable
            msg_cmd.mode[3] = msg->buttons[3];
        }

        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "lx:%.2f ly:%.2f ri:%.2f dn:%.2f rf:%.2f thb:%d thf:%d M/C:%d N/H:%d R/F:%d CB:%d",
                                            msg_cmd.data[0]/32768.0f,msg_cmd.data[1]/32768.0f,msg_cmd.data[2]/32768.0f,
                                            msg_cmd.data[3]/32768.0f,msg_cmd.data[4]/32768.0f,msg_cmd.data[6],msg_cmd.data[5],
                                            msg_cmd.mode[0],msg_cmd.mode[1],msg_cmd.mode[2],msg_cmd.mode[3]);
            publisher_->publish(msg_cmd);
        }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConvertJoy2cmd>());
    rclcpp::shutdown();
    return 0;
}