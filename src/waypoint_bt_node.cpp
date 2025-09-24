#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/SetBool.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h> 
#include <fmt/core.h>

#include <geometry_msgs/Point.h>
#include <vector>
#include <fstream>
#include <sstream>


// Add this structure after your includes
struct Waypoint {
    double x, y, z;
    std::string id;
    
    Waypoint() : x(0), y(0), z(0), id("") {}
    Waypoint(double x_, double y_, double z_, std::string id_) 
        : x(x_), y(y_), z(z_), id(id_) {}
};


class WaypointManager : public BT::SyncActionNode
{
private:
    std::vector<Waypoint> waypoints_;
    int current_index_;
    bool initialized_;

public:
    WaypointManager(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), current_index_(0), initialized_(false)
    {}

    // Method to set waypoints from external source
    void setWaypoints(const std::vector<Waypoint>& waypoints) {
        waypoints_ = waypoints;
        current_index_ = 0;
        initialized_ = false;
    }

   static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::vector<Waypoint>>("waypoints", "Waypoints to manage"),
            BT::InputPort<bool>("waypoint_reached", "Signal that current waypoint was reached"),
            BT::OutputPort<Waypoint>("current_waypoint", "Current waypoint to navigate to"),
            BT::OutputPort<int>("waypoint_index", "Current waypoint index"),
            BT::OutputPort<bool>("has_more_waypoints", "Whether there are more waypoints")
        };
    }
    BT::NodeStatus tick() override
    {
        // Load waypoints from blackboard if not already loaded
        if (waypoints_.empty()) {
            // Try to get waypoints from input port first
            auto input_waypoints = getInput<std::vector<Waypoint>>("waypoints");
            if (input_waypoints) {
                waypoints_ = input_waypoints.value();
                ROS_INFO("Received %zu waypoints from input port", waypoints_.size());
            } 
            // else {
            //     // Fallback: try to get from blackboard directly
            //     auto blackboard = config().blackboard;
            //     if (blackboard && blackboard->getAny("waypoints")) {
            //         waypoints_ = blackboard->get<std::vector<Waypoint>>("waypoints");
            //         ROS_INFO("Received %zu waypoints from blackboard", waypoints_.size());
            //     } else {
            //         ROS_ERROR("No waypoints provided in input port or blackboard!");
            //         return BT::NodeStatus::FAILURE;
            //     }
            // }
        }

        // Initialize outputs on first run
        if (!initialized_) {
            setOutput("current_waypoint", waypoints_[current_index_]);
            setOutput("waypoint_index", current_index_);
            setOutput("has_more_waypoints", current_index_ < waypoints_.size() - 1);
            initialized_ = true;
            ROS_INFO("Initialized with waypoint %d: (%.1f, %.1f, %.1f)", 
                    current_index_, waypoints_[current_index_].x, 
                    waypoints_[current_index_].y, waypoints_[current_index_].z);
            return BT::NodeStatus::SUCCESS;
        }

        // Rest of your existing logic...
        auto waypoint_reached = getInput<bool>("waypoint_reached");
        if (waypoint_reached && waypoint_reached.value()) {
            ROS_INFO("Waypoint %d reached! Moving to next waypoint.", current_index_);
            
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



// Add a node to print current waypoint info
class PrintWaypoint : public BT::SyncActionNode
{
private:
ros::NodeHandle nh_;
ros::Publisher chase_point_pub_;

public:
    PrintWaypoint(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), nh_() 
        {
            // Create publisher for chase point
            chase_point_pub_ = nh_.advertise<geometry_msgs::Point>("/chase_point", 1);
            ROS_INFO("PrintWaypoint: Publishing to /chase_point topic");
        }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<Waypoint>("waypoint", "Waypoint to print"),
            BT::InputPort<int>("index", "Waypoint index")
        };
    }

    BT::NodeStatus tick() override
    {
        auto waypoint = getInput<Waypoint>("waypoint");
        auto index = getInput<int>("index");
        
        if (waypoint && index) {
            std::cout << "Current waypoint [" << index.value() << "]: " 
                    << "(" << waypoint->x << ", " << waypoint->y << ", " << waypoint->z << ") "
                    << "ID: " << waypoint->id << std::endl;
            
            geometry_msgs::Point chase_point;
            chase_point.x = waypoint->x;
            chase_point.y = waypoint->y;
            chase_point.z = waypoint->z;
            
            chase_point_pub_.publish(chase_point);
            
            ROS_INFO("Published waypoint [%d] to /chase_point: (%.2f, %.2f, %.2f)", 
                     index.value(), chase_point.x, chase_point.y, chase_point.z);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};





//-------------------------------------------------------------
class GoalStatusServer : public BT::SyncActionNode
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
            ROS_INFO("Waypoint reached confirmation received");
        } else {
            waypoint_reached_ = false;
            res.success = true;
            res.message = "Waypoint status reset";
        }
        return true;
    }

public:
    GoalStatusServer(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), nh_()
    {
        service_server_ = nh_.advertiseService("/goal_reached", 
            &GoalStatusServer::goalReachedCallback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("waypoint_reached", "Whether waypoint was reached")
        };
    }

    BT::NodeStatus tick() override
    {
        setOutput("waypoint_reached", waypoint_reached_.load());
        
        if (waypoint_reached_) {
            waypoint_reached_ = false; // Reset for next waypoint
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::FAILURE; // Not reached yet
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
            ROS_INFO("Loaded waypoint: %s (%.1f, %.1f, %.1f)", 
                     wp.id.c_str(), wp.x, wp.y, wp.z);
        }
    } else {
        ROS_ERROR("Failed to load waypoints from parameters");
        return -1;
    }
    // Create a BehaviorTreeFactory
    BT::BehaviorTreeFactory factory;


    factory.registerNodeType<GoalStatusServer>("GoalStatusServer");
    factory.registerNodeType<WaypointManager>("WaypointManager");
    factory.registerNodeType<PrintWaypoint>("PrintWaypoint");

    auto blackboard = BT::Blackboard::create();
    blackboard->set("waypoints", waypoints);

    // Get the path to the XML file
    std::string package_path = ros::package::getPath("behaviortree_ros");  // Replace with your actual package name
    std::string xml_file_path = package_path + "/config/nav.xml";
    ROS_INFO("Loading behavior tree from: %s", xml_file_path.c_str());


    auto tree = factory.createTreeFromFile(xml_file_path, blackboard);


    //BT::StdCoutLogger logger_cout(tree);
    ROS_INFO("Starting waypoint behavior tree...");
    ROS_INFO("Call 'rosservice call /goal_reached \"data: true\"' to mark waypoints as reached");
    


    ros::Rate rate(100); //  Hz
    while (ros::ok())
    {
        BT::NodeStatus status = tree.tickRoot();
        
        if (status == BT::NodeStatus::SUCCESS)
        {
            ROS_INFO("Waypoint reached!");
            // You might want to continue or break here
        }
        else if (status == BT::NodeStatus::FAILURE)
        {
            ROS_DEBUG("Waiting for waypoint...");
        }
        
        ros::spinOnce(); // Process ROS callbacks
        rate.sleep();
    }
    
    return 0;
}
