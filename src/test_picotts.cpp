#include <picotts/picotts_publisher.h>

int main(int argc, char *argv[])
{
//  if (argc == 1) {
//    ROS_INFO("Synospis: %s \"STRING\"", argv[0]);
//    return 0;
//  }
  ros::init(argc, argv, "test_picotts");
  PicoTTSPublisher pub;
  while (ros::ok()) {
    sleep(1);
    printf("Your sentence > ");
    std::string sentence;
    std::getline(std::cin, sentence);
    if (sentence.empty())
      break;
    pub.say(sentence);
    ros::spinOnce();
  }
  return 0;
}
