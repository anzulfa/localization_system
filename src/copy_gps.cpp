#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    // pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
    gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps_data", 10);

    //Topic you want to subscribe
    // sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
    gps_sub = n.subscribe("/fix", 10, &SubscribeAndPublish::gpsCallback, this);
  }

  void gpsCallback(const sensor_msgs::NavSatFix &gps_msg)
  {
    // PUBLISHED_MESSAGE_TYPE output;
    sensor_msgs::NavSatFix gps;
    //.... do something with the input and generate the output...
    gps = gps_msg;
  	gps.header.frame_id = "gps_copy";
  	gps_pub.publish(gps);
    // pub_.publish(output);
  }

private:
  ros::NodeHandle n;
  ros::Publisher gps_pub;
  ros::Subscriber gps_sub;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "copy_gps");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
