// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <vector>

const double MIN_SAFE_DISTANCE = 0.5; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        //angle_min = -0.785398; (-45 degrees)
        angle_max_ = laser_scan.angle_max;
        //angle_max = 0.785398; (45 degrees)
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        //ping_indices_[] = 208 - 458 (45 to -45)(box in front of robot)
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
    }
    
   ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   
   // A vector of pings (expanding the sample set)
   std::vector<float> pings_dists_in_front_;
   // For each ping in specified range (45 to -45 - a box), initialize
   for(int index = 300; index <= 360; index++){
   	pings_dists_in_front_.push_back(laser_scan.ranges[index]);
   }
   
   // Set the laser_alarm and then, for each ping, if dist less than min.. FLAG it.
   laser_alarm_=false;
   for(float dists : pings_dists_in_front_){
   	if (dists<MIN_SAFE_DISTANCE) {
        	ROS_WARN("WALL IN FRONT OF ROBOT!!");
       	laser_alarm_=true;
   	}
   }
   
   // Report metrics to us via the terminal
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   ROS_INFO("angle increment = %f",angle_increment_);
   ROS_INFO("laserscan ranges.size() = %f",(double)laser_scan.ranges.size());
   ROS_INFO("laser_alarm state = %f", (double) laser_alarm_);
   
   // Uncomment if needed - this is original code for simple alarm
   //if (ping_dist_in_front_<MIN_SAFE_DISTANCE) {
   //   ROS_WARN("DANGER, WILL ROBINSON!!");
   //    laser_alarm_=true;
   //}
   //else {
   //    laser_alarm_=false;
   //}
   
   // Construct message and publish it
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   
   // Uncomment to enable lidar distance publish
   // Probably don't need the distance message - alarm will be enough (per the bounding box design)
   //std_msgs::Float32 lidar_dist_msg;
   //lidar_dist_msg.data = ping_dist_in_front_;
   //lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    
    //Uncomment to enable lidar distance publish
    //ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    //lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

