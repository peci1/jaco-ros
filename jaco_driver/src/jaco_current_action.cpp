#include <kinova/KinovaTypes.h>

namespace jaco {

JacoCurrentPublisher::JacoCurrentPublisher(JacoComm &arm_comm, const ros::NodeHandle &nh)
    : arm_comm_(arm_comm),
      node_handle_(nh, "current"),
{
    double tolerance;
    node_handle_.param<double>("stall_interval_seconds", stall_interval_seconds_, 0.5);
    node_handle_.param<double>("stall_threshold", stall_threshold_, 1.0);
    node_handle_.param<double>("rate_hz", rate_hz_, 10.0);
    node_handle_.param<double>("tolerance", tolerance, 2.0);
    tolerance_ = static_cast<float>(tolerance);

    action_server_.start();
}


JacoCurrentPublisher::~JacoCurrentPublisher()
{
}


void JacoCurrentPublisher::actionCallback()
{
    joint_current_publisher_ = node_handle_.advertise<jaco_msgs::JointAngles>("out/joint_angles", 2);
}
















}
