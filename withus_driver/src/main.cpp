#include <memory>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "withus_driver/withus_driver_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;

    try{
        auto node = std::make_shared<withus_driver::WithusDriverNode>();    
        rclcpp::spin(node);
    }catch(const std::exception& error){
        RCLCPP_ERROR(
            rclcpp::get_logger("withus_driver"),
            "Withus driver error: %s",
            error.what()
        );
        exit_code = 1;
        
    }
   
    rclcpp::shutdown();
    return exit_code;
}