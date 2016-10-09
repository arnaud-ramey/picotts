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
// picotts
#include "picotts/cached_files_map.h"


enum Engine {
  ENGINE_AT,
  ENGINE_ESPEAK,
  ENGINE_FESTIVAL,
  ENGINE_GNUSTEP,
  ENGINE_GOOGLE, // online
  ENGINE_IVONA, // online
  ENGINE_MARYTTS,
  ENGINE_MICROSOFT, // online
  ENGINE_PICO2WAVE,
  ENGINE_SPEECH_DISPATCHER
};

std::string _scripts_folder = ros::package::getPath("picotts") + "/scripts";
Engine _engine;
utils::Language _language = "en";
std::vector<std::string> _versions;
// ENGINE_AT
static const std::string TMP_AT_FILE = "/tmp/picotts_at.amr";
CachedFilesMap _at_cache(ros::package::getPath("picotts") + "/data/AT_cache/index.csv");
std::string _at_access_token;
// ENGINE_GOOGLE
static const std::string TMP_GOOGLE_FILE = "/tmp/picotts_google.mp3";
CachedFilesMap _google_cache(ros::package::getPath("picotts") + "/data/google_cache/index.csv");
// ENGINE_IVONA
std::string _ivona_credentials = "credentials.txt";
static const std::string TMP_IVONA_FILE = "/tmp/EttsIvona.mp3";
CachedFilesMap _ivona_cache(ros::package::getPath("picotts") + "/data/ivona_cache/index.csv");
// ENGINE_MARYTTS
static const std::string TMP_MARYTTS_FILE = "/tmp/picotts_marytts.wav";
CachedFilesMap _marytts_cache(ros::package::getPath("picotts") + "/data/marytts_cache/index.csv");
// ENGINE_MICROSOFT
static const std::string TMP_MICROSOFT_FILE = "/tmp/picotts_microsoft.wav";
const std::string MICROSOFT_API_KEY = "6844AE3580856D2EC7A64C79F55F11AA47CB961B";
CachedFilesMap _microsoft_cache(ros::package::getPath("picotts") + "/data/microsoft_cache/index.csv");
// ENGINE_PICO2WAVE
static const std::string TMP_PICO2WAVE_FILE = "/tmp/picotts_pico2wave.wav";
CachedFilesMap _pico2wave_cache(ros::package::getPath("picotts") + "/data/pico2wave_cache/index.csv");

////////////////////////////////////////////////////////////////////////////////

bool mplayer_file(const std::string & filename)  {
  //ROS_INFO("mplayer_file(%s)", filename.c_str());
  std::ostringstream play_cmd;
  // read it with mplayer
  play_cmd.str("");
  play_cmd << "mplayer " << filename << " -really-quiet ";
  play_cmd << " 2> /dev/null ";
  ROS_INFO("running command '%s'", play_cmd.str().c_str());
  return (utils::exec_system(play_cmd.str().c_str()) == 0);
}

////////////////////////////////////////////////////////////////////////////////

//! \return true if file cached and could be played
bool mplayer_cached_file_if_available(CachedFilesMap & cache,
                                      const std::string & key) {
  std::string cached_file;
  if (!cache.get_cached_file(key, cached_file)) {
    //    ROS_INFO("sentence '%s' was not in cache:'%s'",
    //           key.c_str(), cached_file.c_str());
    return false;
  }
  ROS_INFO("sentence '%s' was already in cache:'%s'",
           key.c_str(), cached_file.c_str());
  return mplayer_file(cached_file);
}

////////////////////////////////////////////////////////////////////////////////

//! for sentence less than MAX_CHUNK_SIZE characters
bool at_get_short_chunk(const std::string & langAt,
                        const std::string & sentence,
                        const std::string & filename_out) {
  printf("at_get_short_chunk('%s':'%s' (size:%i))\n",
         langAt.c_str(), sentence.c_str(), (int) sentence.size());
  // clean the sentence
  std::string sentence_clean = sentence;

  if (_at_access_token.empty()) {
    std::string CLIENT_ID="14ys9fpee45ih7mr4he6in5kgwmnyxuw",
        CLIENT_SECRET_KEY="cgezjftvo5ynif9x55rsbu3s32jkowgc";

    //getting the access_token
    std::ostringstream command;
    std::string responseFile = "/tmp/reply_at_cloud_service.txt";
    command << "curl https://api.att.com/oauth/v4/token --request POST --insecure ";
    //command << " -o " << responseFile;
    command << "--data \"client_id="<< CLIENT_ID << "&client_secret="<< CLIENT_SECRET_KEY
            << "&grant_type=client_credentials&scope=TTS\" > " << responseFile;
    ROS_INFO("running command '%s'", command.str().c_str());
    if (utils::exec_system(command.str().c_str()))
      return false;

    int pos = 0;
    std::string responseContent;
    std::ifstream csv_content(responseFile.c_str());
    if (!csv_content.is_open() || !std::getline (csv_content,responseContent)) {
      ROS_WARN("Could not read file '%s'", responseFile.c_str());
      return false;
    }
    ROS_INFO("Response of authorisation key: %s", responseContent.c_str());
    _at_access_token = utils::extract_from_tags
        (responseContent, "\"access_token\":\"","\",\"",pos);
    if (_at_access_token.empty()) {
      ROS_WARN("Could not get access token from AT&T");
      return false;
    }
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
  command2 << " --request POST > \"" << filename_out << "\"";

  ROS_INFO("running command '%s'", command2.str().c_str());
  utils::exec_system(command2.str().c_str());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void tts_cb(const std_msgs::StringConstPtr & msg) {
  // split sentence and find correct language
  utils::stringSplit(msg->data, "|", &_versions);
  std::string tosay = "";
  if (!utils::find_given_language_in_multilanguage_line(_versions, _language, tosay))
    tosay = msg->data;
  std::string key = _language + ":" + tosay;
  // clean sentence
  std::string tosay_clean = tosay;
  utils::find_and_replace(tosay_clean, "\"", "\\\"");
  // generate command
  std::ostringstream command;

  if (_engine == ENGINE_AT) {
    if (mplayer_cached_file_if_available(_at_cache, key))
      return;
    std::string langAt = "en-US";
    if (_language == "es")      langAt = "es-US";
    else if (_language == "en") langAt = "en-US";
    else {
      ROS_WARN("Unsupported language'%s'", _language.c_str());
    }
    if (at_get_short_chunk(langAt, tosay_clean, TMP_AT_FILE)
        && mplayer_file(TMP_AT_FILE))
      _at_cache.add_cached_file(key, TMP_AT_FILE);
  } // end if (ENGINE_AT)
  else if (_engine == ENGINE_ESPEAK) {
    // list of languages: $ espeak --voices
    // to sort them:      $ espeak --voices | awk '{ print $2 }' | sort
    // https://tuxicoman.jesuislibre.net/2015/05/synthese-vocale-sous-linux.html
    // $ espeak -v mb/mb-fr1 -s 120 "Bonjour, je parle le français aussi bien que vous. Ou presque."
    command << "espeak ";
    if (_language == "fr" || _language == "fr-FR")
      command << "-v mb/mb-fr1 -s 120";
    else
      command << "-v " << _language;
    command << " \"" << tosay_clean << "\"";
    ROS_INFO("running command '%s'", command.str().c_str());
    utils::exec_system(command.str().c_str());
  } // end if (ENGINE_ESPEAK)
  else if (_engine == ENGINE_FESTIVAL) {
    // list languages: $ festival --language xxx
    command << "echo \"" << tosay_clean << "\" | festival --tts --lang=" << _language;
    ROS_INFO("running command '%s'", command.str().c_str());
    utils::exec_system(command.str().c_str());
  } // end if (ENGINE_FESTIVAL)
  else if (_engine == ENGINE_GNUSTEP) {
    command << "say \"" << tosay_clean << "\"";
    ROS_INFO("running command '%s'", command.str().c_str());
    utils::exec_system(command.str().c_str());
  } // end if (ENGINE_GNUSTEP)
  else if (_engine == ENGINE_GOOGLE) {
    // list languages: https://translate.google.com/#en/af/test and check for "loudspeaker" button
    if (mplayer_cached_file_if_available(_google_cache, key))
      return;
    // build the url
    command << "wget ";
    command << "\"http://translate.google.com/translate_tts?";
    command << "tl=" << _language;
    command << "&q=" << tosay_clean;
    command << "&ie=UTF-8&total=1&idx=0&client=uc3m";
    command << "\"";
    command << " --output-document=" << TMP_GOOGLE_FILE;
    command << " --no-verbose";
    // trick Google API with no referrer
    // cf http://www.askapache.com/dreamhost/wget-header-trick.html
    command << " --referer=\"\"";
    command << " --user-agent=\"Mozilla/5.0 (Windows; U; Windows NT 5.1; "
               "en-US; rv:1.8.1.6) Gecko/20070725 Firefox/2.0.0.6\"";
    ROS_INFO("running command '%s'", command.str().c_str());
    if (utils::exec_system(command.str().c_str()) == 0
        && mplayer_file(TMP_GOOGLE_FILE))
      _google_cache.add_cached_file(key, TMP_GOOGLE_FILE);
  } // end if (ENGINE_GOOGLE)
  else if (_engine == ENGINE_IVONA) {
    // list languages: https://www.ivona.com/us/about-us/voice-portfolio/
    if (mplayer_cached_file_if_available(_ivona_cache, key))
      return;
    std::string lang_ivona = "en-GB", voicename = "Amy";
    if (_language == "en") {
      lang_ivona = "en-GB";
      voicename = "Amy";
    } else if (_language == "es") {
      lang_ivona = "es-ES";
      voicename = "Conchita";
    } else if (_language == "fr") {
      lang_ivona = "fr-FR";
      voicename = "Celine";
    } else {
      ROS_WARN("Unsupported language'%s'", _language.c_str());
    }
    command << "bash " << _scripts_folder << "/ivona.bash \""
            << _ivona_credentials << "\"  \""
            << tosay_clean << "\"  \""
            << lang_ivona << "\"  \""
            << voicename << "\"";
    ROS_INFO("running command '%s'", command.str().c_str());
    if (utils::exec_system(command.str().c_str()) == 0
        && mplayer_file(TMP_IVONA_FILE))
      _ivona_cache.add_cached_file(key, TMP_IVONA_FILE);
  } // end if (ENGINE_IVONA)
  else if (_engine == ENGINE_MARYTTS) {
    // list languages: http://localhost:59125/locales: de en_GB en_US fr it lb ru sv te tr
    if (mplayer_cached_file_if_available(_marytts_cache, key))
      return;
    command << "wget ";
    command << "\"http://localhost:59125/process?INPUT_TYPE=TEXT";
    command << "&OUTPUT_TYPE=AUDIO";
    command << "&AUDIO_OUT=WAVE_FILE";
    command << "&AUDIO=WAVE_FILE";
    if (_language == "en")
      command << "&LOCALE=en-US";
    else
      command << "&LOCALE=" << _language;
    if (_language == "en")
      command << "&VOICE=cmu-slt-hsmm"; // en_US female hmm
    else if (_language == "fr")
      command << "&VOICE=upmc-pierre-hsmm"; // fr male hmm
    else {
      ROS_WARN("Unsupported language'%s'", _language.c_str());
      command << "&VOICE=cmu-slt-hsmm"; // en_US female hmm
    }
    command << "&INPUT_TEXT=" << tosay_clean;
    command << "\"";
    command << " --output-document=" << TMP_MARYTTS_FILE;
    command << " --quiet";
    if (utils::exec_system(command.str().c_str()) == 0
        && mplayer_file(TMP_MARYTTS_FILE))
      _marytts_cache.add_cached_file(key, TMP_MARYTTS_FILE);
  } // end if (ENGINE_ENGINE_MARYTTS)
  else if (_engine == ENGINE_MICROSOFT) {
    // http://api.microsofttranslator.com/v2/Http.svc/Speak?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B&text=Hello,%20my%20friend!&language=en
    // list of languages:
    // http://api.microsofttranslator.com/v2/Http.svc/GetLanguagesForSpeak?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B
    if (mplayer_cached_file_if_available(_microsoft_cache, key))
      return;
    std::ostringstream command;
    command << "wget ";
    command << "\"http://api.microsofttranslator.com/v2/Http.svc/Speak?";
    command << "appId=" << MICROSOFT_API_KEY;
    command << "&language=" << _language;
    command << "&text=" << tosay_clean;
    command << "\"";
    command << " --output-document=" << TMP_MICROSOFT_FILE;
    command << " --no-verbose";
    if (utils::exec_system(command.str().c_str()) == 0
        && mplayer_file(TMP_MICROSOFT_FILE))
      _microsoft_cache.add_cached_file(key, TMP_MICROSOFT_FILE);
  } // end if (ENGINE_MICROSOFT)
  else if (_engine == ENGINE_PICO2WAVE) {
    // https://tuxicoman.jesuislibre.net/2015/05/synthese-vocale-sous-linux.html
    // $ pico2wave -l fr-FR -w test.wav "Bonjour, je parle le français aussi bien que vous. Ou presque."
    // list languages: $ pico2wave --lang xxx -w foo.wav "ok"
    if (mplayer_cached_file_if_available(_pico2wave_cache, key))
      return;
    std::string lang_pico2wave = "en-GB";
    if (_language == "de")      lang_pico2wave = "de-DE";
    else if (_language == "es") lang_pico2wave = "es-ES";
    else if (_language == "en") lang_pico2wave = "en-GB";
    else if (_language == "fr") lang_pico2wave = "fr-FR";
    else if (_language == "it") lang_pico2wave = "it-IT";
    else {
      ROS_WARN("Unsupported language'%s'", _language.c_str());
    }
    command << "pico2wave "
            << "  --lang=" << lang_pico2wave
            << "  --wave=" << TMP_PICO2WAVE_FILE
            << "  \"" << tosay_clean << "\"";
    ROS_INFO("running command '%s'", command.str().c_str());
    if (utils::exec_system(command.str().c_str()) == 0
        && mplayer_file(TMP_PICO2WAVE_FILE))
      _pico2wave_cache.add_cached_file(key, TMP_PICO2WAVE_FILE);
  } // end if (ENGINE_PICO2WAVE)
  else if (_engine == ENGINE_SPEECH_DISPATCHER) {
    // list languages: $ spd-say --list-synthesis-voices | awk ' { print $2 } ' | sort -u | tr '\n' ' '
    command << "spd-say";
    command << " --language " << _language;
    command << " \"" << tosay_clean << "\"";
    ROS_INFO("running command '%s'", command.str().c_str());
    utils::exec_system(command.str().c_str());
  } // end if (ENGINE_SPEECH_DISPATCHER)
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
  } else if (engine_lower.find("mary") != std::string::npos) {
    _engine = ENGINE_MARYTTS;
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
  std::string engine_str = "pico";
  nh_public.param("language", _language, _language);
  nh_private.param("engine", engine_str, engine_str);
  nh_private.param("ivona_credentials", _ivona_credentials, _ivona_credentials);
  engine_switcher(engine_str);
  // make subscribers
  ros::Subscriber tts_sub = nh_public.subscribe("tts", 1, tts_cb);
  ros::Subscriber engine_sub = nh_private.subscribe("engine", 1, engine_cb);
  ROS_INFO("language:'%s', engine:'%s', listening to:'%s'",
           _language.c_str(), engine_str.c_str(), tts_sub.getTopic().c_str());
  ros::spin();
}
