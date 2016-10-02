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
#include <fstream>
#include <vector>
#include <string>
// ROS msg
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

enum Engine {
  ENGINE_AT,
  ENGINE_ESPEAK,
  ENGINE_FESTIVAL,
  ENGINE_GNUSTEP,
  ENGINE_GOOGLE, // online
  ENGINE_IVONA, // online
  ENGINE_MICROSOFT, // online
  ENGINE_PICO2WAVE,
  ENGINE_SPEECH_DISPATCHER
};

typedef std::string Language;
static const std::string TMPWAV = "/tmp/picotts.wav";
static const std::string TMPAMR = "/tmp/picotts.amr";
std::string _scripts_folder = ros::package::getPath("picotts") + "/scripts";
std::string _at_access_token;
std::string _ivona_credentials = "credentials.txt";
Engine _engine;
Language _language = "en";
std::vector<std::string> _versions;

////////////////////////////////////////////////////////////////////////////////

/*! \brief   split a std::string
 * \param   str the long std::string
 * \param   delim the std::string which is the separator between words, for instance '/'
 * \param   results a pointer towards a vector of std::string, will contain the results
 */
inline void stringSplit(const std::string & str, const std::string & delim,
                        std::vector<std::string>* results) {
  results->clear();
  if (str == "")
    return;
  size_t delim_pos, search_pos = 0;
  while (search_pos <= str.size() - 1) {
    delim_pos = str.find(delim, search_pos);
    if (delim_pos == std::string::npos) { // no more delim
      results->push_back(str.substr(search_pos));
      return;
    }
    if (delim_pos > 0) // == 0 only happens if str starts with delim
      results->push_back(str.substr(search_pos, delim_pos - search_pos));
    search_pos = delim_pos + delim.size();
    // quit if we reached the end of the std::string or std::string empty
  }
} // end std::stringSplit();

////////////////////////////////////////////////////////////////////////////////

std::string extract_from_tags(const std::string & content,
                              const std::string & block_begin,
                              const std::string & block_end,
                              int & initial_search_pos) {
  // find the beginning
  size_t pos_begin = content.find(block_begin, initial_search_pos);
  if (pos_begin == std::string::npos) {
    ROS_DEBUG("block_begin '%s' could not be found", block_begin.c_str());
    return "";
  }
  // remove the block at the beginning
  pos_begin += block_begin.length();
  // find the end
  size_t pos_end = content.find(block_end, pos_begin);
  if (pos_end == std::string::npos) {
    ROS_DEBUG("block_end '%s' could not be found", block_end.c_str());
    return "";
  }
  // update the index
  initial_search_pos = pos_end + block_end.length();
  // extract the substring
  std::string ans = content.substr(pos_begin, pos_end - pos_begin);
  return ans;
}

////////////////////////////////////////////////////////////////////////////////

inline bool extract_language(const std::string & sentence,
                             Language & ans) {
  size_t sep_pos = sentence.find(':');
  if ( sep_pos == std::string::npos )
    return false;
  ans = sentence.substr(0, sep_pos);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool find_given_language_in_multilanguage_line(
    const std::vector<std::string> & versions,
    const Language target_language,
    std::string & ans) {
  // check in each version if it was found
  Language current_language;
  std::vector<std::string> good_versions;
  for(std::vector<std::string>::const_iterator current_version = versions.begin();
      current_version != versions.end() ; ++current_version) {
    if (!extract_language(*current_version, current_language)
        || current_language != target_language)
      continue;
    // we remove the prefix and the colon
    good_versions.push_back(current_version->substr(1 + target_language.size()));
  } // end loop versions
  if (good_versions.empty())
    return false;
  ans = good_versions.at(rand() % good_versions.size());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

//! for sentence less than MAX_CHUNK_SIZE characters
void at_say_short_chunk(const std::string & langAt,
                        const std::string & sentence) {
  printf("at_say_short_chunk('%s':'%s' (size:%i))\n",
         langAt.c_str(), sentence.c_str(), (int) sentence.size());
  // clean the sentence
  std::string sentence_clean = sentence;

  if (_at_access_token.empty()) {
    std::string CLIENT_ID="14ys9fpee45ih7mr4he6in5kgwmnyxuw";
    std::string CLIENT_SECRET_KEY="cgezjftvo5ynif9x55rsbu3s32jkowgc";

    //getting the access_token
    std::ostringstream command;
    std::string responseFile = "/tmp/respuse_at_cloud_service.txt";
    command << "curl https://api.att.com/oauth/v4/token --request POST --insecure ";
    //command << " -o " << responseFile;
    command << "--data \"client_id="<< CLIENT_ID << "&client_secret="<< CLIENT_SECRET_KEY << "&grant_type=client_credentials&scope=SPEECH,STTC,TTS\"" << ">" << responseFile;
    ROS_INFO("picotts: running command '%s'", command.str().c_str());
    system(command.str().c_str());

    int pos = 0;
    std::string responseContent;
    //std::stringUtils::retrieve_file(responseFile, responseContent);
    std::ifstream csv_content(responseFile.c_str());
    if (!csv_content.is_open() || !std::getline (csv_content,responseContent)) {
      ROS_WARN("Could not read file '%s'", responseFile.c_str());
      return;
    }
    ROS_INFO("Response of authoriation key: %s", responseContent.c_str());
    _at_access_token = extract_from_tags
        (responseContent, "\"access_token\":\"","\",\"",pos);
    ROS_INFO("Access_token: %s",_at_access_token.c_str());
  } // end if (_at_access_token.empty())

  std::ostringstream command2;
  std::string URL="https://api.att.com";
  command2 << "curl " << URL << "/speech/v3/textToSpeech";
  command2 << " --header \"Authorization: Bearer "<< _at_access_token <<" \"";
  command2 << " --header \"Accept: audio/amr\"";
  command2 << " --header \"Content-Type: text/plain\"";
  command2 << " --header \"X-Arg: VoiceName=";
  if (langAt == "es-US")
    command2 << "rosa";  // "rosa" or "alberto"
  else
    command2 << "crystal";  //"crystal" or "mike"
  command2 << ",Volume=100\"";
  command2 << " --header \"Content-Language: " << langAt <<"\"";
  command2 << " --data \"" << sentence_clean << "\"";
  command2 << " --request POST > \"" << TMPAMR << "\"";

  ROS_INFO("picotts: running command '%s'", command2.str().c_str());
  system(command2.str().c_str());

  std::ostringstream play_cmd;
  std::string curr_amr = TMPAMR;
  // read it with mplayer
  play_cmd.str("");
  play_cmd << "mplayer " << curr_amr << " -really-quiet ";
  play_cmd << " 2> /dev/null ";
  ROS_INFO("picotts: running command '%s'", play_cmd.str().c_str());
  system(play_cmd.str().c_str());
}

////////////////////////////////////////////////////////////////////////////////

void tts_cb(const std_msgs::StringConstPtr & msg) {
  // split sentence and find correct language
  stringSplit(msg->data, "|", &_versions);
  std::string tosay = "";
  if (!find_given_language_in_multilanguage_line(_versions, _language, tosay))
    tosay = msg->data;
  // generate command
  std::ostringstream command;
  if (_engine == ENGINE_AT) {
    at_say_short_chunk("en-US", tosay);
  }else if (_engine == ENGINE_ESPEAK) {
    command << "espeak \"" << tosay << "\"";
  } else if (_engine == ENGINE_FESTIVAL) {
    command << "echo \"" << tosay << "\" | festival --tts";
  } else if (_engine == ENGINE_GNUSTEP) {
    command << "say \"" << tosay << "\"";
  } else if (_engine == ENGINE_GOOGLE) {
    ROS_ERROR("ENGINE_GOOGLE not implemented"); return;
  } else if (_engine == ENGINE_IVONA) {
    command << "bash " << _scripts_folder << "/ivona.bash \""
            << _ivona_credentials << "\" \"" << tosay << "\"";
  } else if (_engine == ENGINE_MICROSOFT) {
    ROS_ERROR("ENGINE_MICROSOFT not implemented"); return;
  } else if (_engine == ENGINE_PICO2WAVE) {
    command << "pico2wave --wave=" << TMPWAV << " \"" << tosay
            << "\"; aplay " << TMPWAV << " --quiet";
  } else if (_engine == ENGINE_SPEECH_DISPATCHER) {
    command << "spd-say \"" << tosay << "\"";
  }
  // execute command
  if (!command.str().empty()) {
    ROS_INFO("picotts: running command '%s'", command.str().c_str());
    system(command.str().c_str());
  }
} // end tts_cb()

////////////////////////////////////////////////////////////////////////////////

bool engine_switcher(const std::string & engine_str) {
  std::string engine_lower = engine_str;
  std::transform(engine_lower.begin(), engine_lower.end(), engine_lower.begin(), ::tolower);
  if (engine_lower.find("at") != std::string::npos) {
    _engine = ENGINE_AT;
  } else if (engine_lower.find("espeak") != std::string::npos) {
    _engine = ENGINE_ESPEAK;
  } else if (engine_lower.find("festival") != std::string::npos) {
    _engine = ENGINE_FESTIVAL;
  } else if (engine_lower.find("gnustep") != std::string::npos) {
    _engine = ENGINE_GNUSTEP;
  } else if (engine_lower.find("google") != std::string::npos) {
    _engine = ENGINE_GOOGLE;
  } else if (engine_lower.find("ivona") != std::string::npos) {
    _engine = ENGINE_IVONA;
  } else if (engine_lower.find("microsoft") != std::string::npos) {
    _engine = ENGINE_MICROSOFT;
  } else if (engine_lower.find("pico") != std::string::npos) {
    _engine = ENGINE_PICO2WAVE;
  } else if (engine_lower.find("dispatch") != std::string::npos) {
    _engine = ENGINE_SPEECH_DISPATCHER;
  }
  else {
    ROS_WARN("Unknown engine '%s'", engine_str.c_str());
    return false;
  }
  ROS_INFO("picotts now using engine %i (%s)", _engine, engine_lower.c_str());
  return true;
} // end engine_switcher()

void engine_cb(const std_msgs::StringConstPtr & msg) {
  engine_switcher(msg->data);
} // end engine_cb()

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  srand(time(NULL));
  ros::init(argc, argv, "picotts"); //Initialise and create a ROS node
  ros::NodeHandle nh_public, nh_private("~");
  // get params
  nh_public.param("language", _language, _language);
  std::string engine_str = "pico";
  nh_private.param("engine", engine_str, engine_str);
  engine_switcher(engine_str);
  // make subscribers
  ros::Subscriber tts_sub = nh_public.subscribe("tts", 1, tts_cb);
  ros::Subscriber engine_sub = nh_private.subscribe("engine", 1, engine_cb);
  ros::spin();
}
