#include <ros/ros.h>
#include <std_msgs/String.h>
extern "C" {
#include <libnotify/notify.h>
}

// https://wiki.archlinux.org/index.php/Desktop_notifications
void msg_cb(const std_msgs::String & msg_raw) {
  std::ostringstream msg;
  msg << "\"" << msg_raw.data << "\"";
  NotifyNotification * notif = notify_notification_new
      (ros::this_node::getName().c_str(), msg.str().c_str(), "dialog-information");
  notify_notification_show (notif, NULL);
  g_object_unref(G_OBJECT(notif));
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tts2notify");
  notify_init ("tts2notify");
  ros::NodeHandle nh_public;
  ros::Subscriber tts_sub = nh_public.subscribe("tts", 1, msg_cb);
  std_msgs::String msg; msg.data = "tts2notify started.";
  msg_cb(msg);
  ros::spin();
  notify_uninit();
  return 0;
}
