/*!
  \file         picotts.cpp
  \author       Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date         2016/09/29

  ______________________________________________________________________________

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  ______________________________________________________________________________
*/
// C++
#include <ostream>
// ROS msg
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

enum Engine {
  ENGINE_AT, // online
  ENGINE_ESPEAK,
  ENGINE_FESTIVAL,
  ENGINE_GNUSTEP,
  ENGINE_GOOGLE, // online
  ENGINE_IVONA, // online
  ENGINE_MICROSOFT, // online
  ENGINE_PICO2WAVE,
  ENGINE_SPEECH_DISPATCHER
};

Engine engine = ENGINE_PICO2WAVE;
std::string scripts_folder = ros::package::getPath("picotts") + "/scripts";
std::string ivona_credentials = "credentials.txt";
std::string TMPWAV = "/tmp/picotts.wav";

////////////////////////////////////////////////////////////////////////////////

void tts_cb(const std_msgs::StringConstPtr & msg) {
  std::ostringstream order;
  std::string tosay = msg->data;
  if (engine == ENGINE_AT) {
    ROS_ERROR("Not implemented");
  } else if (engine == ENGINE_ESPEAK) {
    order << "espeak \"" << tosay << "\"";
  } else if (engine == ENGINE_FESTIVAL) {
    order << "echo \"" << tosay << "\" | festival --tts";
  } else if (engine == ENGINE_GNUSTEP) {
    order << "say \"" << tosay << "\"";
  } else if (engine == ENGINE_GOOGLE) {
    ROS_ERROR("Not implemented");
  } else if (engine == ENGINE_IVONA) {
    order << "bash " << scripts_folder << "/ivona.bash \""
          << ivona_credentials << "\" \"" << tosay << "\"";
  } else if (engine == ENGINE_MICROSOFT) {
    ROS_ERROR("Not implemented");
  } else if (engine == ENGINE_PICO2WAVE) {
    order << "pico2wave --wave=" << TMPWAV << "\"" << tosay
          << "\"; aplay " << TMPWAV;
  } else if (engine == ENGINE_SPEECH_DISPATCHER) {
    order << "spd-say \"" << tosay << "\"";
  }
} // end tts_cb()

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  ros::init(argc, argv, "picotts"); //Initialise and create a ROS node
  ros::NodeHandle nh_public, nh_private("~");
  ros::Subscriber tts_sub = nh_public.subscribe("tts", 1, tts_cb);
  ros::spin();
}
