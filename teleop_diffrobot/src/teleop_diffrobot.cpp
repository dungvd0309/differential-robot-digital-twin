#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <iostream>
#include <map>
#include <iomanip>

class TeleopCmdVel1 : public rclcpp::Node
{
public:
    TeleopCmdVel1()
    : Node("teleop_cmdvel1"),
      speed_(0.5),
      turn_(1.0)
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
        printInstructions();
        printStatus();
    }

    void spin()
    {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (rclcpp::ok())
        {
            char key = getKey();
            geometry_msgs::msg::Twist twist;

            if (moveBindings_.count(key))
            {
                auto m = moveBindings_[key];
                twist.linear.x = m[0] * speed_;
                twist.angular.z = m[1] * turn_;
            }
            else if (speedBindings_.count(key))
            {
                auto s = speedBindings_[key];
                speed_ *= s[0];
                turn_  *= s[1];
                printStatus();
                continue;
            }
            else
            {
                if (key == '\x03' || key == 'q') // Ctrl+C hoặc q -> thoát
                {
                    std::cout << "Thoát teleop..." << std::endl;
                    break;
                }
                // phím khác -> dừng
            }

            pub_->publish(twist);
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    double speed_, turn_;

    std::map<char, std::array<double, 2>> moveBindings_ = {
        {'i', {1, 0}},    // tiến
        {',', {-1, 0}},   // lùi
        {'j', {0, 1}},    // quay trái
        {'l', {0, -1}},   // quay phải
        {'k', {0, 0}}     // dừng
    };

    std::map<char, std::array<double, 2>> speedBindings_ = {
        {'q', {1.1, 1.1}}, {'z', {0.9, 0.9}},
        {'w', {1.1, 1.0}}, {'x', {0.9, 1.0}},
        {'e', {1.0, 1.1}}, {'c', {1.0, 0.9}}
    };

    char getKey()
    {
        fd_set set;
        struct timeval timeout;
        int rv;
        char buff = 0;

        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 0.1s

        rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
        if (rv > 0)
            read(STDIN_FILENO, &buff, 1);
        return buff;
    }

    void printInstructions()
    {
        std::cout << R"(
This node takes keypresses from the keyboard and publishes them
as geometry_msgs/Twist messages to /cmd_vel1.
It works best with a US keyboard layout.

---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C or q to quit
)" << std::endl;
    }

    void printStatus()
    {
        std::cout << "currently:\tspeed " << std::fixed << std::setprecision(2)
                  << speed_ << "\tturn " << turn_ << std::endl;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopCmdVel1>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}

