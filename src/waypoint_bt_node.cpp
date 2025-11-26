#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/SetBool.h> 
#include <dji_srvs/SetString.h> // custom service type, built from src/dji_srvs
#include <dji_srvs/SetValue.h>  // custom service type, built from src/dji_srvs
#include <std_msgs/Float64.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h> 

#include <fmt/core.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <vector>
#include <fstream>
#include <sstream>

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>

#include <tf/tf.h>
// Add this structure after your includes
// struct Waypoint {
//     double x, y, z;
//     std::string id;
    
//     Waypoint() : x(0), y(0), z(0), id("") {}
//     Waypoint(double x_, double y_, double z_, std::string id_) 
//         : x(x_), y(y_), z(z_), id(id_) {}
// };

struct Waypoint {
   double x, y, z;
   double yaw; // New field
   std::string id;
  
   Waypoint() : x(0), y(0), z(0), yaw(0), id("") {}
   Waypoint(double x_, double y_, double z_, double yaw_, std::string id_)
       : x(x_), y(y_), z(z_), yaw(yaw_), id(id_) {}
};



struct WaypointReached {
    std::string id;
    bool reached;
    
    WaypointReached() : id(""), reached(false) {}
    WaypointReached(std::string id_, bool reached_) 
        : id(id_), reached(reached_) {}
};


//-------------------------
class RosSubscriptionManager : public BT::SyncActionNode
{

private:

    ros::NodeHandle* nh_;
    std::mutex mutex_;  // Protect member variables


    ros::Subscriber tagslam_heartbeat_sub_;
    std::string tagslam_heartbeat_topic_name_ = "/tagslam/heartbeat";
    std_msgs::Header tagslam_heartbeat_msg_;
    
    ros::Subscriber tag_detector_sub_;
    std::string tag_detector_topic_name_ = "/detector/tags";
    apriltag_msgs::ApriltagArrayStamped tag_detector_msg_;


    ros::Subscriber tagslam_odom_sub_;
    nav_msgs::Odometry tagslam_odom_msg_;
    std::string tagslam_odom_topic_name_ = "/tagslam/odom/body_rig";

    void tagslamOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        tagslam_odom_msg_ = *msg;
    }


    void heartbeatCallback(const std_msgs::Header::ConstPtr& msg) {
        tagslam_heartbeat_msg_ = *msg;
    }

    void tagArrayCallback(const apriltag_msgs::ApriltagArrayStamped::ConstPtr& msg) {
        tag_detector_msg_ = *msg;
    }

public:
    RosSubscriptionManager(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config),
        nh_(nullptr)
    {
        nh_ = config.blackboard->get<ros::NodeHandle*>("node_handle");
        tagslam_heartbeat_sub_ = nh_->subscribe(tagslam_heartbeat_topic_name_, 10, 
                            &RosSubscriptionManager::heartbeatCallback, this);
        tag_detector_sub_ = nh_->subscribe(tag_detector_topic_name_, 10,
                            &RosSubscriptionManager::tagArrayCallback, this);
       
        tagslam_odom_sub_ = nh_->subscribe(tagslam_odom_topic_name_, 10,
                            &RosSubscriptionManager::tagslamOdomCallback, this);

    }
    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std_msgs::Header>("heartbeat"),
            BT::OutputPort<apriltag_msgs::ApriltagArrayStamped>("tags"),
            BT::OutputPort<nav_msgs::Odometry>("tagslam_odom")
        };
    }

    BT::NodeStatus tick() override
    {

            std::lock_guard<std::mutex> lock(mutex_);
            setOutput("heartbeat", tagslam_heartbeat_msg_);
            setOutput("tags", tag_detector_msg_);
            setOutput("tagslam_odom", tagslam_odom_msg_);
        return BT::NodeStatus::SUCCESS;
    }
};

class GoalManager : public BT::StatefulActionNode
{ 
public:
    GoalManager(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {}
    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<nav_msgs::Odometry>("tagslam_odom"),
            BT::InputPort<Waypoint>("current_waypoint"),
            BT::OutputPort<WaypointReached>("waypoint_reached", "Signal that current waypoint was reached"),
        };
    }


    BT::NodeStatus onStart() override
    {
        
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override
    {
        auto odom = getInput<nav_msgs::Odometry>("tagslam_odom");
        auto goal = getInput<Waypoint>("current_waypoint");

        if (!odom) {
            ROS_WARN("GoalManager: Missing port input [tagslam_odom]");
            return BT::NodeStatus::FAILURE;
        }
        if (!goal) {
            ROS_WARN("GoalManager: Missing port input [current_waypoint]");
            return BT::NodeStatus::FAILURE;
        }
        if (odom->header.stamp.sec == 0 && odom->header.stamp.nsec == 0) {
            ROS_WARN("GoalManager: tagslam_odom not yet received");
            return BT::NodeStatus::RUNNING;
        }
        std::string goal_id = goal->id;
        double x_goal = goal->x;
        double y_goal = goal->y;
        double z_goal = goal->z;
        double x_odom = odom->pose.pose.position.x;
        double y_odom = odom->pose.pose.position.y;
        double z_odom = odom->pose.pose.position.z;

        double distance = sqrt(pow(x_goal - x_odom, 2) + 
                               pow(y_goal - y_odom, 2) + 
                               pow(z_goal - z_odom, 2));
        // ROS_INFO("GoalManager: Distance to goal: %.3f m", distance);

        if (distance < 0.05) {
            ROS_INFO("GoalManager: Distance to goal %s: %.3f m", goal_id.c_str(), distance);
            
            setOutput("waypoint_reached", WaypointReached(goal_id, true));
        } else {
            setOutput("waypoint_reached", WaypointReached(goal_id, false));
        }
        return BT::NodeStatus::SUCCESS;
    }
    void onHalted() override
    {
        ROS_ERROR("GoalManager halted");
    }
};



//-------------------------------
class IsTagslamHealthy : public BT::ConditionNode
{
public:
    IsTagslamHealthy(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() 
    {
        return {
            BT::InputPort<std_msgs::Header>("heartbeat"),
            BT::InputPort<apriltag_msgs::ApriltagArrayStamped>("tags"),
            BT::OutputPort<bool>("is_healthy", "Indicates if Tagslam is healthy")
        };
    }
 
    BT::NodeStatus tick() override
    {
        auto heartbeat = getInput<std_msgs::Header>("heartbeat");
        if (!heartbeat) {
            ROS_ERROR("IsTagslamHealthy: Missing required port input [heartbeat]");
            return BT::NodeStatus::FAILURE;
        }
        auto tags = getInput<apriltag_msgs::ApriltagArrayStamped>("tags");
        if (!tags) {
            ROS_ERROR("IsTagslamHealthy: Missing required port input [tags]");
            return BT::NodeStatus::FAILURE;
        }

        if (tags->apriltags.empty()) {
            setOutput("is_healthy", false);
            ROS_WARN("IsTagslamHealthy: No tags detected");
            return BT::NodeStatus::FAILURE;
        }

        ros::Time current_time = ros::Time::now();
        double heartbeat_age = (current_time - heartbeat->stamp).toSec();
        if (heartbeat_age > 0.25) { //  seconds timeout
            setOutput("is_healthy", false);
            ROS_WARN("IsTagslamHealthy: Heartbeat too old (%.2f seconds)", heartbeat_age);
            return BT::NodeStatus::FAILURE;
        }
        setOutput("is_healthy", true);
        return BT::NodeStatus::SUCCESS;
    }
private:

};




//-------------------------------
class SleepNode : public BT::StatefulActionNode
{
public:
    SleepNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("msec", "Sleep duration in milliseconds")
        };
    }

    BT::NodeStatus onStart() override
    {
        int msec = 0;
        getInput("msec", msec);
        
        if (msec <= 0) {
            // No need to sleep
            return BT::NodeStatus::SUCCESS;
        }
        
        // Set deadline for when sleep should end
        deadline_ = std::chrono::system_clock::now() + 
                    std::chrono::milliseconds(msec);
        ROS_INFO("Sleeping for %d milliseconds", msec);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        
        
        // Check if deadline reached
        if (std::chrono::system_clock::now() >= deadline_) {
            ROS_INFO("Sleep completed");
            return BT::NodeStatus::SUCCESS;
        }
        
        // Still sleeping
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        ROS_INFO("Sleep interrupted");
    }

private:
    std::chrono::system_clock::time_point deadline_;
};
//---------------------




//----------------
class PublishWaypoint : public BT::SyncActionNode
{
private:
    ros::NodeHandle* nh_;
    // ros::Publisher chase_point_pub_;
    // geometry_msgs::Point chase_point;

    ros::Publisher chase_pose_pub_;
    geometry_msgs::Pose chase_pose;

    

public:
    PublishWaypoint(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config),
        nh_(nullptr) 
    {
        nh_ = config.blackboard->get<ros::NodeHandle*>("node_handle");

        // Create publisher for chase point
        // chase_point_pub_ = nh_->advertise<geometry_msgs::Point>("/chase_point", 1, true);
        // ROS_INFO("PublishWaypoint: Publishing to /chase_point topic");
        chase_pose_pub_ = nh_->advertise<geometry_msgs::Pose>("/chase_pose", 1, true);
        ROS_INFO("PublishWaypoint: Publishing to /chase_pose topic");
    }
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<Waypoint>("current_waypoint", "Current waypoint to navigate to"),
        };
    }
    BT::NodeStatus tick() override
    {
        auto waypoint = getInput<Waypoint>("current_waypoint");
        if (!waypoint) {
            ROS_ERROR("PublishWaypoint: Missing required input [current_waypoint]");
            return BT::NodeStatus::FAILURE;
        }
        // chase_point.x = waypoint->x;
        // chase_point.y = waypoint->y;
        // chase_point.z = waypoint->z;
        // chase_point_pub_.publish(chase_point);
        // ROS_INFO("Published waypoint: (%.1f, %.1f, %.1f)", chase_point.x, chase_point.y, chase_point.z);
    
        chase_pose.position.x = waypoint->x;
        chase_pose.position.y = waypoint->y;
        chase_pose.position.z = waypoint->z;
        tf::Quaternion q;
        q.setRPY(0, 0, waypoint->yaw);
        chase_pose.orientation.x = q.x();
        chase_pose.orientation.y = q.y();
        chase_pose.orientation.z = q.z();
        chase_pose.orientation.w = q.w();
        chase_pose_pub_.publish(chase_pose);
        ROS_INFO("Published waypoint: (%.1f, %.1f, %.1f, %.1f)", chase_pose.position.x, chase_pose.position.y, chase_pose.position.z, waypoint->yaw);

        return BT::NodeStatus::SUCCESS;   
    }
};





//----------------
class WaypointManager : public BT::SyncActionNode
{
private:
    std::vector<Waypoint> waypoints_;
    int current_index_;
    bool initialized_;

    // Method to set waypoints from external source
    void setWaypoints(const std::vector<Waypoint>& waypoints) {
        waypoints_ = waypoints;
        current_index_ = 0;
        initialized_ = false;
    }


public:
    WaypointManager(const std::string& name, const BT::NodeConfiguration& config,
                    std::vector<Waypoint> myWaypoints): 
        BT::SyncActionNode(name, config), 
        current_index_(0), 
        initialized_(false)
        
    {
        setWaypoints(myWaypoints);
    }

    

   static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("segment_completed", "Whether segment is completed"),
            BT::OutputPort<Waypoint>("current_waypoint", "Current waypoint to navigate to"),
            BT::OutputPort<int>("waypoint_count", "Total number of waypoints"),
            BT::OutputPort<int>("waypoint_index", "Current waypoint index"),
            BT::OutputPort<bool>("has_more_waypoints", "Whether there are more waypoints")
        };
    }
    BT::NodeStatus tick() override
    {
        // Reset waypoint reached output
        
        // Initialize outputs on first run
        if (!initialized_) {
            setOutput("current_waypoint", waypoints_[current_index_]);
            setOutput("waypoint_index", current_index_);
            setOutput("waypoint_count", static_cast<int>(waypoints_.size()));
            setOutput("has_more_waypoints", current_index_ < waypoints_.size() - 1);
            ROS_INFO("has_more_waypoints: %d", current_index_ < waypoints_.size() - 1);
            initialized_ = true;
            ROS_INFO("Initialized with waypoint %d: (%.1f, %.1f, %.1f)", 
                    current_index_, waypoints_[current_index_].x, 
                    waypoints_[current_index_].y, waypoints_[current_index_].z);
            return BT::NodeStatus::SUCCESS;
        }




        auto segment_completed = getInput<bool>("segment_completed");
        // increment waypoint if reached
        // ROS_INFO("WaypointManager: segment_completed=%d", segment_completed.value());
        if (segment_completed.value()) {
            
            
            ROS_INFO("Segment %d completed! Moving to next waypoint.", current_index_);
            
            current_index_++;
            if (current_index_ >= waypoints_.size()) {
                ROS_INFO("All waypoints completed!");
                setOutput("has_more_waypoints", false);
                return BT::NodeStatus::SUCCESS;
            }
            
            
            setOutput("current_waypoint", waypoints_[current_index_]);
            setOutput("waypoint_index", current_index_);
            setOutput("has_more_waypoints", current_index_ < waypoints_.size() - 1);
            
            ROS_INFO("New target waypoint %d: (%.1f, %.1f, %.1f) [%s]", 
                    current_index_, waypoints_[current_index_].x, 
                    waypoints_[current_index_].y, waypoints_[current_index_].z,
                    waypoints_[current_index_].id.c_str());
        }

        return BT::NodeStatus::SUCCESS;
    }
};
//---------------




class EnableTrajGen : public BT::SyncActionNode
{
private:
    ros::NodeHandle* nh_;
    ros::Publisher trajgen_enable_pub_;
    std_msgs::Bool msg_;

public:
    EnableTrajGen(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config),
        nh_(nullptr)
    {   
        nh_ = config.blackboard->get<ros::NodeHandle*>("node_handle"); 
        trajgen_enable_pub_ = nh_->advertise<std_msgs::Bool>("/trajGen/enable", 1, true);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("enable_trajgen", "Enable or disable trajectory generation"),
        };
    }

    BT::NodeStatus tick() override
    {
        auto enable = getInput<bool>("enable_trajgen");
        if (!enable) {
            ROS_ERROR("enableTrajGen: Missing required input [enable_trajgen]");
            return BT::NodeStatus::FAILURE;
        }
        // Here you would add the logic to enable/disable trajectory generation
        if (enable.value()) {
            msg_.data = true;
            trajgen_enable_pub_.publish(msg_);
            ROS_INFO("Trajectory generation enabled");
        } else {
            msg_.data = false;
            trajgen_enable_pub_.publish(msg_);
            ROS_INFO("Trajectory generation disabled");
        }
        return BT::NodeStatus::SUCCESS;
    }
};
//---------------       

 
//---------------
class ApproachWaypoint : public BT::StatefulActionNode
{
private:
    ros::NodeHandle* nh_;

    ros::Publisher trajgen_enable_pub_;
    std_msgs::Bool trajgen_enable_msg_;
public:
    ApproachWaypoint(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config),
        nh_(nullptr) 

    {
        nh_ = config.blackboard->get<ros::NodeHandle*>("node_handle");
        trajgen_enable_pub_ = nh_->advertise<std_msgs::Bool>("/trajGen/enable", 1, true);
    }    

    static BT::PortsList providedPorts()
    {
        return 
        { 
            BT::InputPort<Waypoint>("current_waypoint", "Current waypoint to navigate to"),
            BT::InputPort<bool>("is_healthy", "Indicates if Tagslam is healthy"),
            BT::InputPort<WaypointReached>("waypoint_reached", "Signal that current waypoint was reached") 
        };
    }

    BT::NodeStatus onStart() override
    {
        

        auto waypoint = getInput<Waypoint>("current_waypoint");
        if (!waypoint) {
            ROS_ERROR("PublishWaypoint: Missing required input [current_waypoint]");
            return BT::NodeStatus::FAILURE;
        }
        auto waypoint_reached_ = getInput<WaypointReached>("waypoint_reached");
        if (!waypoint_reached_) {
            ROS_ERROR("ApproachWaypoint: Missing required input [waypoint_reached]");
            return BT::NodeStatus::FAILURE;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (!getInput<bool>("is_healthy").value())
        {
            trajgen_enable_msg_.data = false;
            trajgen_enable_pub_.publish(trajgen_enable_msg_);
            ROS_ERROR("ApproachWaypoint: onRunning, Tagslam is not healthy!");
            return BT::NodeStatus::FAILURE;
        }
        //ROS_INFO("ApproachWaypoint: navigating...");

        auto waypoint_reached_ = getInput<WaypointReached>("waypoint_reached");
        if (waypoint_reached_.value().reached)
        {
            trajgen_enable_msg_.data = false;
            trajgen_enable_pub_.publish(trajgen_enable_msg_);
            ROS_INFO("ApproachWaypoint: Waypoint reached!");
            ROS_INFO("Trajectory generation DISABLED in ApproachWaypoint");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        trajgen_enable_msg_.data = false;
        trajgen_enable_pub_.publish(trajgen_enable_msg_);
        ROS_WARN("ApproachWaypoint halted!");
    }
};



//----------------------------------
class PerformAction : public BT::StatefulActionNode
{
private:
    ros::NodeHandle* nh_;
    ros::ServiceClient client_;
    ros::Subscriber sub_;
    dji_srvs::SetString srv_;  

    std_msgs::String latest_msg_;
    bool msg_received_ = false;

    void topicCallback(const std_msgs::String::ConstPtr& msg)
    {
        latest_msg_ = *msg;
        msg_received_ = true;
        // TODO: Add processing logic for the received message
        //...or maybe not...
        // ROS_INFO("I heard: [%s]", msg->data.c_str());
    }


public:
    PerformAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), 
        nh_(nullptr) 
    {
        nh_ = config.blackboard->get<ros::NodeHandle*>("node_handle");
        client_ = nh_->serviceClient<dji_srvs::SetString>(getInput<std::string>("service_name").value());
        sub_ = nh_->subscribe(getInput<std::string>("topic_name").value(), 10, 
                             &PerformAction::topicCallback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("service_name"),
            BT::InputPort<std::string>("request_data"),
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<std::string>("success_condition"),
            BT::OutputPort<std::string>("received_data")
        };
    }

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;
};
BT::NodeStatus PerformAction::onStart() 
{
    ROS_INFO("Starting service call");
    
    // Send async service request
    srv_.request.value = getInput<std::string>("request_data").value();  
    client_.call(srv_);
    msg_received_ = false;  
    return BT::NodeStatus::RUNNING;  // Continue to onRunning()
}
BT::NodeStatus PerformAction::onRunning()
{
   

    if (!msg_received_) { return BT::NodeStatus::RUNNING;}
    ROS_INFO("Processing received message: %s", latest_msg_.data.c_str());
    setOutput("received_data", latest_msg_.data);
    auto success_condition = getInput<std::string>("success_condition").value();
    if (latest_msg_.data == success_condition) {return BT::NodeStatus::SUCCESS;}
    else {return BT::NodeStatus::FAILURE;}


    return BT::NodeStatus::RUNNING;
}


void PerformAction::onHalted()
{
  printf("[ PerformAction: ABORTED ]");
}








//-----------

class StartSegment : public BT::SyncActionNode
{
public:
    StartSegment(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            
            BT::OutputPort<bool>("segment_completed", "Whether segment is completed")
        };
    }


    BT::NodeStatus tick() override
    {
        setOutput("segment_completed", false);
        return BT::NodeStatus::SUCCESS;
    }

};


class EndSegment : public BT::SyncActionNode
{
public:
    EndSegment(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("segment_completed", "Whether segment is completed")
        };
    }


    BT::NodeStatus tick() override
    {


        setOutput("segment_completed", true);
        ROS_INFO("EndSegment: Segment completed!");
        return BT::NodeStatus::SUCCESS;


    }

};
//---------------









//---------------



class StateTracker : public BT::StatefulActionNode
{
public:
    StateTracker(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("end_condition", "condition to end the state"),
        };
    }

    BT::NodeStatus onStart() override
    {
        return calculateNodeStatus();
    }

    BT::NodeStatus onRunning() override
    {
        return calculateNodeStatus();
    }

    virtual BT::NodeStatus calculateNodeStatus()
    {
        auto end_condition = getInput<bool>("end_condition").value();
        if (end_condition)  // Simulate completion after 5 ticks
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        ROS_WARN("StateTracker halted!");
    }
};


class MissionStateTracker : public StateTracker
{

public:
    MissionStateTracker(const std::string& name, const BT::NodeConfiguration& config)
        : StateTracker(name, config)
    {}
    BT::NodeStatus calculateNodeStatus() override
    {
        auto end_condition = getInput<bool>("end_condition").value();
        if (!end_condition)  // Simulate completion after 5 ticks
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

 };


class GoalStateTracker : public StateTracker
{

public:
    GoalStateTracker(const std::string& name, const BT::NodeConfiguration& config)
        : StateTracker(name, config)
    {}
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<WaypointReached>("waypoint_reached", "condition to end the goal"),
            BT::InputPort<Waypoint>("current_waypoint", "Current waypoint to navigate to"),
        };
    }
    BT::NodeStatus calculateNodeStatus() override
    {
        auto waypoint_reached_id = getInput<WaypointReached>("waypoint_reached").value().id;
        auto current_waypoint_id = getInput<Waypoint>("current_waypoint").value().id;
        auto is_reached = getInput<WaypointReached>("waypoint_reached").value().reached;
        bool end_condition = (waypoint_reached_id == current_waypoint_id) && is_reached;
        if (end_condition)  // Simulate completion after 5 ticks
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

 };









int main(int argc, char** argv) { 

    // Initialize ROS
    ros::init(argc, argv, "waypoint_bt_node");
    ros::NodeHandle nh("~");



    std::vector<Waypoint> waypoints;
    XmlRpc::XmlRpcValue waypoint_list;
    if (nh.getParam("waypoints", waypoint_list)) {
        ROS_INFO("Loading %d waypoints from parameters", waypoint_list.size());
        
        for (int i = 0; i < waypoint_list.size(); i++) {
            XmlRpc::XmlRpcValue& waypoint_data = waypoint_list[i];
            
            Waypoint wp;
            wp.x = static_cast<double>(waypoint_data["x"]);
            wp.y = static_cast<double>(waypoint_data["y"]);
            wp.z = static_cast<double>(waypoint_data["z"]);
            wp.id = static_cast<std::string>(waypoint_data["id"]);
            
            waypoints.push_back(wp);
            // ROS_INFO("Loaded waypoint: %s (%.1f, %.1f, %.1f)", 
            //          wp.id.c_str(), wp.x, wp.y, wp.z);
        }
    } else {
        ROS_ERROR("Failed to load waypoints from parameters");
        return -1;
    }
    // Create a BehaviorTreeFactory
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder builder_A =
    [waypoints](const std::string& name, const BT::NodeConfiguration& config)
    {
        return std::make_unique<WaypointManager>(name, config, waypoints);
    };
    // Register custom nodes
    factory.registerNodeType<StartSegment>("StartSegment");
    factory.registerNodeType<EndSegment>("EndSegment");
    factory.registerNodeType<ApproachWaypoint>("ApproachWaypoint");
    factory.registerBuilder<WaypointManager>("WaypointManager", builder_A);
    factory.registerNodeType<MissionStateTracker>("MissionStateTracker");
    factory.registerNodeType<GoalStateTracker>("GoalStateTracker");
    factory.registerNodeType<PerformAction>("PerformAction");
    factory.registerNodeType<SleepNode>("SleepNode");
    factory.registerNodeType<PublishWaypoint>("PublishWaypoint");
    factory.registerNodeType<EnableTrajGen>("EnableTrajGen");
    factory.registerNodeType<RosSubscriptionManager>("RosSubscriptionManager");
    factory.registerNodeType<IsTagslamHealthy>("IsTagslamHealthy");
    factory.registerNodeType<GoalManager>("GoalManager");


    // Create the blackboard and set initial values
    auto blackboard = BT::Blackboard::create();
    blackboard->set("topic_name", "/Mavic_3M/state/other");
    blackboard->set("service_name", "/Mavic_3M/call_API");
    blackboard->set("segment_completed", false); //must initialize segment_completed to false 
    blackboard->set("node_handle", &nh);  
    // Get the path to the XML file
    std::string package_path = ros::package::getPath("behaviortree_ros");  // Replace with your actual package name
    std::string xml_file_path = package_path + "/config/nav.xml";
    ROS_INFO("Loading behavior tree from: %s", xml_file_path.c_str());


    auto tree = factory.createTreeFromFile(xml_file_path, blackboard);

    // Create the PublisherZMQ for Groot visualization
    unsigned max_msg_per_second = 25;   // Adjust rate as needed
    unsigned publisher_port = 1666;     // Default Groot publisher port
    unsigned server_port = 1667;        // Default Groot server port
    BT::PublisherZMQ publisher_zmq(tree, max_msg_per_second, 
                                    publisher_port, server_port);
    

    // Create the StdCoutLogger for console logging
    //BT::StdCoutLogger logger_cout(tree);
    ROS_INFO("ZMQ Publisher started. Connect Groot to localhost:%d", publisher_port);
    ROS_INFO("Starting waypoint behavior tree...");
    ROS_INFO("Call 'rosservice call /goal_reached \"data: true\"' to mark waypoints as reached");
    


    //ros::Rate rate(100); //  Hz
    BT::NodeStatus status = tree.tickRoot();
    while (ros::ok()) // && status == BT::NodeStatus::RUNNING
    {
            
 
        
        ros::spinOnce(); // Process ROS callbacks
        
        // change to tree sleep, 10hz. formally rate.sleep();
        tree.sleep(std::chrono::milliseconds(100));

        BT::NodeStatus status = tree.tickRoot();
        if (status == BT::NodeStatus::SUCCESS)
        {
            ROS_INFO("Behavior tree completed successfully!");
            break;
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            ROS_WARN("Behavior tree failed!");
            break;
        }
    }

   

    return 0;
}
