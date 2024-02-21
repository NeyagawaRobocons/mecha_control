#include "rclcpp/rclcpp.hpp"

#include "cmd_seq_class.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MechSeqNode>());
    rclcpp::shutdown();
    return 0;
}
