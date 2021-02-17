// This node: check a predetermined-size box in front of the robot and publish whether if there is any obsticle in it.
#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <vector>
#include <cmath>

// these values to be set within the laser callback
int setup = -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
int centerIndex = -1; //! NOT real. Update within callback

// box's size
//! please choose farEdge and nearEdge carefully.
const float width = 0.4;            // half the width of the rectangular box
const float farEdge = 0.7;          // distance to further edge
const float nearEdge = 0.4;         // distance to closer edge (there should be a min value for this)
const double centerAngle = 0;  // angle of from the robot to the center of the box vs. robot's heading
int leftCornerIndex, rightCornerIndex;
double leftCornerAngle, rightCornerAngle;

ros::Publisher lidar_alarm_front_publisher_;
ros::Publisher lidar_dist_publisher_;

// Helper functions
// Convert angle to index for the laser scan
int angle2Index(double angle) {
    //fron_indices = 208 - 458 (45 to -45) (box in front of robot)
    int index = (angle - angle_min_)/angle_increment_;
    ROS_INFO("%i",index);
    return index;
}

// Convert index to angle for the laser scan
double index2Angle(int index) {
    double angle = index * angle_increment_ + angle_min_;
    return angle;
}

// Slicing the vector of the wanted windows
std::vector<float> vecSlice(std::vector<float> &v, int m, int n)    {
    std::vector<float> vec;
    for (int i=m; i<n+1; i++) 
        vec.push_back(v[i]); 
    return vec;
}

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    // setup the call back function for the first message.
    if (setup < 0)  {
        // Lidar's params
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        centerIndex = angle2Index(0.0) + 1;

        // Box's params
        //! make sure that the index within the set range.
        rightCornerAngle = atan2(-width/2, nearEdge) + centerAngle;
        leftCornerAngle = atan2(width/2, nearEdge) + centerAngle;
        rightCornerIndex = angle2Index(rightCornerAngle);
        leftCornerIndex = angle2Index(leftCornerAngle);
        
        // Complete setting up
        setup = 1;
    }

// vector of ranges to the point within the box
std::vector<float> laserRange(laser_scan.ranges);
std::vector<float> boxRange = vecSlice(laserRange,rightCornerIndex,leftCornerIndex);
std::vector<float> boxCordX(boxRange), boxCordY(boxRange);
int centerBoxIndex = (int) (boxRange.size()/2);

laser_alarm_=false;
float dist, minDist=10.0;
float sumX = 0, sumY = 0;
for (int i=0; i < boxRange.size(); i++){
    boxCordX[i] = boxRange[i]*cos(index2Angle(i + centerIndex- centerBoxIndex));
    boxCordY[i] = boxRange[i]*sin(index2Angle(i + centerIndex - centerBoxIndex));
    sumX += boxCordX[i];
    sumY += boxCordY[i];
    
    dist = pow(boxCordX[i],2) + pow(boxCordY[i],2);
    if (dist < minDist) minDist=dist;
    
    if (boxCordX[i] <= farEdge && boxCordX[i] >= nearEdge) {
        if (std::fabs(boxCordY[i]) <= (width/2)){
            laser_alarm_=true;
        }
    }
}

// Report metrics to us via the terminal
ROS_INFO("left = %f",(double) leftCornerIndex);
ROS_INFO("right = %f",(double) rightCornerIndex);
ROS_INFO("averageX of boxRange = %f",(sumX/boxRange.size()));
ROS_INFO("averageY of boxRange = %f",(sumY/boxRange.size()));
ROS_INFO("laser_alarm state = %f", (double) laser_alarm_);
ROS_INFO("closest obstacle distance = %f", (double) dist);
ROS_INFO("centerAngle = %f", centerAngle);
ROS_INFO("boxRange.size = %f",(double) boxRange.size());
ROS_INFO("------------------------------");

// Construct message and publish it
std_msgs::Bool lidar_alarm_msg;
lidar_alarm_msg.data = laser_alarm_;
lidar_alarm_front_publisher_.publish(lidar_alarm_msg);

//! Uncomment to enable lidar distance publish
//! Probably don't need the distance message - alarm will be enough (per the bounding box design)
//! std_msgs::Float32 lidar_dist_msg;
//! lidar_dist_msg.data = ping_dist_in_front_;
//! lidar_dist_publisher_.publish(lidar_dist_msg); 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("robot0/lidar_alarm_front", 1);
    lidar_alarm_front_publisher_ = pub; // let's make this global, so callback can use it
    
    //!Uncomment to enable lidar distance publish
    //!ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    //!lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    
    ros::spin();    // this is essentially a "while(1)" statement, except it
                    // forces refreshing wakeups upon new data arrival
                    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

