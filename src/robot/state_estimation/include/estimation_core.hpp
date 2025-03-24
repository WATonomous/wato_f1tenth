#ifndef ESTIMATION_CORE_HPP_
#define ESTIMATION_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class EstimationCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit EstimationCore(const rclcpp::Logger& logger);
    // Default constructor (no parameters)
    EstimationCore() = default;
  private:
    rclcpp::Logger logger_;

};

}  

#endif 