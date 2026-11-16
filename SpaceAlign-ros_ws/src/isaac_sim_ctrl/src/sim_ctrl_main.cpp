#include "isaac_sim_ctrl/SimCtrl.hpp"


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

    auto sim_ctrl_node = std::make_shared<SimCtrl>();
    sim_ctrl_node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sim_ctrl_node);
    executor.spin();

    rclcpp::shutdown();

	return 0;
}
