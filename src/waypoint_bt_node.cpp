#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/SetBool.h> 
#include <dji_srvs/SetString.h> // custom service type, built from src/dji_srvs
#include <dji_srvs/SetValue.h>  // custom service type, built from src/dji_srvs

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h> 
#include <fmt/core.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <vector>
#include <fstream>
#include <sstream>

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <chrono>


// Add this structure after your includes
struct Waypoint {
    double x, y, z;
    std::string id;
    
    Waypoint() : x(0), y(0), z(0), id("") {}
    Waypoint(double x_, double y_, double z_, std::string id_) 
        : x(x_), y(y_), z(z_), id(id_) {}
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
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        
        
        // Check if deadline reached
        if (std::chrono::system_clock::now() >= deadline_) {
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



//----------------
class WaypointManager : public BT::SyncActionNode
{
private:
    std::vector<Waypoint> waypoints_;
    int current_index_;
    bool initialized_;

    ros::NodeHandle nh_;
    ros::Publisher chase_point_pub_;
    geometry_msgs::Point chase_point;
    int last_published_index_; 

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
        initialized_(false),
        last_published_index_(-1), 
        nh_() 
    {
        setWaypoints(myWaypoints);
        // Create publisher for chase point
        chase_point_pub_ = nh_.advertise<geometry_msgs::Point>("/chase_point", 1, true);
        ROS_INFO("PrintWaypoint: Publishing to /chase_point topic");
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
        
        //setOutput("segment_completed", false);
        // Initialize outputs on first run
        if (!initialized_) {
            //setOutput("waypoint_reached", false);
            setOutput("current_waypoint", waypoints_[current_index_]);
            setOutput("waypoint_index", current_index_);
            setOutput("waypoint_count", static_cast<int>(waypoints_.size()));
            setOutput("has_more_waypoints", current_index_ < waypoints_.size() - 1);
            initialized_ = true;
            ROS_INFO("Initialized with waypoint %d: (%.1f, %.1f, %.1f)", 
                    current_index_, waypoints_[current_index_].x, 
                    waypoints_[current_index_].y, waypoints_[current_index_].z);
            return BT::NodeStatus::SUCCESS;
        }


        if (current_index_ != last_published_index_) {
            chase_point.x = waypoints_[current_index_].x;
            chase_point.y = waypoints_[current_index_].y;
            chase_point.z = waypoints_[current_index_].z;
            chase_point_pub_.publish(chase_point);
            
            last_published_index_ = current_index_;
            
            ROS_INFO("WaypointManager: Published waypoint %d to /chase_point: (%.2f, %.2f, %.2f)",
                     current_index_, chase_point.x, chase_point.y, chase_point.z);
        }


        auto segment_completed = getInput<bool>("segment_completed");
        // increment waypoint if reached
        // ROS_INFO("WaypointManager: segment_completed=%d", segment_completed.value());
        if (segment_completed.value()) {
            //setOutput("waypoint_reached", true);
            // waypoint_reached_ = false; // Reset for next waypoint
            
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
class ApproachWaypoint : public BT::StatefulActionNode
{
private:

    ros::NodeHandle nh_;
    ros::ServiceServer service_server_;
    std::atomic<bool> waypoint_reached_{false};

    bool goalReachedCallback(std_srvs::SetBool::Request& req, 
                            std_srvs::SetBool::Response& res)
    {
        if (req.data) {
            waypoint_reached_ = true;
            res.success = true;
            res.message = "Waypoint marked as reached";
            //ROS_INFO("Waypoint reached confirmation received");
        } else {
            waypoint_reached_ = false;
            res.success = true;
            res.message = "Waypoint status reset";
        }
        return true;
    };
public:
    ApproachWaypoint(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config),
        nh_() 
    {
        service_server_ = nh_.advertiseService("/goal_reached", &ApproachWaypoint::goalReachedCallback, this);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<bool>("waypoint_reached", "Signal that current waypoint was reached") };
    }

    BT::NodeStatus onStart() override
    {
        waypoint_reached_ = false;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (waypoint_reached_)
        {
            setOutput("waypoint_reached", true);
            ROS_INFO("ApproachWaypoint: Waypoint reached!");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        ROS_WARN("ApproachWaypoint halted!");
    }
};



//----------------------------------
class PerformAction : public BT::StatefulActionNode
{
private:
    ros::NodeHandle nh_;
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
        // ROS_INFO("I heard: [%s]", msg->data.c_str());
    }


public:
    PerformAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), nh_() 
    {
        client_ = nh_.serviceClient<dji_srvs::SetString>(getInput<std::string>("service_name").value());
        sub_ = nh_.subscribe(getInput<std::string>("topic_name").value(), 10, 
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
    // if (msg_received_) {
    //     ROS_INFO("Processing received message: %s", latest_msg_.data.c_str());
    //     setOutput("received_data", latest_msg_.data);
        
        
    //     return BT::NodeStatus::SUCCESS;
    // } 
    // // Still waiting for message
    // return BT::NodeStatus::RUNNING;

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



class MissionState : public BT::StatefulActionNode
{
public:
    MissionState(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("has_more_waypoints", "has more waypoints"),
        };
    }

    BT::NodeStatus onStart() override
    {
        // auto has_more_waypoints = getInput<bool>("has_more_waypoints");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

        auto has_more_waypoints = getInput<bool>("has_more_waypoints").value();
        if (!has_more_waypoints)  // Simulate completion after 5 ticks
        {
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        ROS_WARN("Mission state halted!");
    }
};












int main(int argc, char** argv) { 

    // Initialize ROS
    ros::init(argc, argv, "waypoint_bt_node");
    ros::NodeHandle nh;
    


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
    factory.registerNodeType<MissionState>("MissionState");
    factory.registerNodeType<PerformAction>("PerformAction");
    factory.registerNodeType<SleepNode>("SleepNode");
    
    // Create the blackboard and set initial values
    auto blackboard = BT::Blackboard::create();
    blackboard->set("topic_name", "/Mavic_3M/state/other");
    blackboard->set("service_name", "/Mavic_3M/call_API");
    blackboard->set("segment_completed", false); //must initialize segment_completed to false 

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
