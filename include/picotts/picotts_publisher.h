#ifndef PICOTTS_PUBLISHER_H
#define PICOTTS_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/String.h>

class PicoTTSPublisher {
public:
  PicoTTSPublisher() {
    _pub = _nh.advertise<std_msgs::String>("tts", 1);
  }

  void say(const std::string & sentence) {
    std_msgs::String msg;
    msg.data = sentence;
    _pub.publish(msg);
  }

protected:
  ros::NodeHandle _nh;
  ros::Publisher _pub;
}; // end class PicoTTSPublisher

#endif // PICOTTS_PUBLISHER_H

