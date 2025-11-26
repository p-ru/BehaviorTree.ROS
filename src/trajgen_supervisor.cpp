#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <signal.h>

class TrajGenSupervisor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber chase_point_sub_;
    ros::Subscriber odom_sub_;
    ros::Timer health_check_timer_;
    
    bool chase_point_healthy_;
    bool odom_healthy_;
    pid_t trajgen_pid_;
    int iteration_count_;
    
public:
    TrajGenSupervisor() : chase_point_healthy_(false), 
                          odom_healthy_(false), 
                          trajgen_pid_(-1),
                          iteration_count_(0) {
        
        // Subscribe to topics
        chase_point_sub_ = nh_.subscribe("/chase_point", 10, 
                                        &TrajGenSupervisor::chasePointCallback, this);
        odom_sub_ = nh_.subscribe("/tagslam/odom/body_rig", 10,
                                 &TrajGenSupervisor::odomCallback, this);
        
        // Create timer for health checks (1 Hz)
        health_check_timer_ = nh_.createTimer(ros::Duration(1.0), 
                                             &TrajGenSupervisor::checkHealth, this);
        
        ROS_INFO("==================================================");
        ROS_INFO("TrajGen Supervisor STARTED");
        ROS_INFO("Monitoring: /chase_point and /tagslam/odom/body_rig");
        ROS_INFO("==================================================");
    }
    
    ~TrajGenSupervisor() {
        if (trajgen_pid_ > 0) {
            kill(trajgen_pid_, SIGTERM);
            ROS_INFO("Killed trajgen on shutdown");
        }
    }
    
    void chasePointCallback(const geometry_msgs::Point::ConstPtr& msg) {
        chase_point_healthy_ = true;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_healthy_ = true;
    }
    
    void checkHealth(const ros::TimerEvent&) {
        iteration_count_++;
        bool both_healthy = chase_point_healthy_ && odom_healthy_;
        
        // Display status
        ROS_INFO("Check #%d - chase_point: %s, odom: %s, trajgen: %s",
                 iteration_count_,
                 chase_point_healthy_ ? "HEALTHY" : "NO DATA",
                 odom_healthy_ ? "HEALTHY" : "NO DATA",
                 trajgen_pid_ > 0 ? "RUNNING" : "STOPPED");
        
        if (both_healthy && trajgen_pid_ <= 0) {
            // Start trajgen
            ROS_WARN(">>> Both topics healthy - STARTING trajgen <<<");
            startTrajGen();
        } 
        else if (!both_healthy && trajgen_pid_ > 0) {
            // Stop trajgen
            ROS_WARN(">>> Topics unhealthy - STOPPING trajgen <<<");
            stopTrajGen();
        }
        
        // Reset health flags
        chase_point_healthy_ = false;
        odom_healthy_ = false;
    }
    
    void startTrajGen() {
        // Use system() which inherits shell environment including ROS variables
        std::string cmd = "rosrun trajgen trajGen &";
        
        // Get PID by launching in a way we can track
        FILE* pipe = popen("rosrun trajgen trajGen > /dev/null 2>&1 & echo $!", "r");
        if (pipe) {
            char buffer[128];
            if (fgets(buffer, sizeof(buffer), pipe) != NULL) {
                trajgen_pid_ = std::atoi(buffer);
                ROS_INFO("Successfully started trajGen with PID: %d", trajgen_pid_);
            }
            pclose(pipe);
        } else {
            ROS_ERROR("Failed to start trajGen");
            trajgen_pid_ = -1;
        }
    }

    void stopTrajGen() {
        if (trajgen_pid_ > 0) {
            // Kill by node name using rosnode
            int result = system("rosnode kill /trajgen_node 2>/dev/null");
            if (result == 0) {
                ROS_INFO("Stopped trajGen via rosnode kill");
            } else {
                // Fallback: kill by PID
                kill(trajgen_pid_, SIGTERM);
                ROS_INFO("Stopped trajGen via PID");
            }
            trajgen_pid_ = -1;
            ros::Duration(0.5).sleep();
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajgen_supervisor");
    
    TrajGenSupervisor supervisor;
    
    ros::spin();
    
    return 0;
}
