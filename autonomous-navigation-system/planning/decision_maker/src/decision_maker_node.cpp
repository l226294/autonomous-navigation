#include "decision_maker/decision_maker.hpp"
#include "decision_maker/factory.hpp"

// using namespace planning;

int main(int argc, char **argv)
{
  Context* context = new Context();
  // create state machine
  Factory::CreateState(context, "Ready");
  Factory::CreateState(context, "LaneKeep");
  Factory::CreateState(context, "OverTake");
  Factory::CreateState(context, "CarFollow");
  Factory::CreateState(context, "Complete");
  Factory::CreateState(context, "Error");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DecisionMakerNode>(context);
  rclcpp::spin(node);
  rclcpp::shutdown();
  if (context)
	{
		delete context;
		context = nullptr;
	}
  return 0;
}
