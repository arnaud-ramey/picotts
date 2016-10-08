picotts
=======

A lightweight ROS package for Text-to-Speech generation.


 - MaryTTS
 [download page](http://mary.dfki.de/download/)
 [direct v5.2 download link](https://github.com/marytts/marytts/releases/download/v5.2/marytts-5.2.zip)

  - List of voices: http://localhost:59125/voices
    Best French voice: "upmc-pierre-hsmm fr male hmm" & "enst-camille-hsmm fr female hmm"
    English: "dfki-spike-hsmm en_GB male hmm"
    "cmu-slt-hsmm en_US female hmm"
    wget:
    http://www.dfki.de/pipermail/mary-users/2011-September/date.html#1029

```bash
$ wget  "http://localhost:59125/process?INPUT_TYPE=TEXT&OUTPUT_TYPE=AUDIO&INPUT_TEXT=Welcome to the world of speech synthesis. I am using wget\!&AUDIO_OUT=WAVE_FILE&LOCALE=en_US&VOICE=cmu-slt-hsmm&AUDIO=WAVE_FILE"  -O /tmp/marytts.wav --quiet
$ aplay /tmp/marytts.wav
```

Licence
=======

BSD


Authors
=======

  - Package maintainer: Arnaud Ramey (arnaud.a.ramey@gmail.com)

Compile and install
===================

ROS Kinetic + catkin
-------------------

Compile with [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make):

```bash
$ rosdep install picotts --ignore-src
$ sudo apt install mbrola mbrola-fr*
$ catkin_make --only-pkg-with-deps picotts
```

Run
===

```bash
$ rosrun picotts picotts.exe
$ rostopic pub /tts std_msgs/String "OK"
$ rostopic pub /tts std_msgs/String "en:Hello|en:Hi"
$ rostopic pub /picotts/engine std_msgs/String "microsoft"
Read time:
$ echo -e "data: \"`date +%T`\"\n---" | rostopic pub /tts std_msgs/String -l
```

Parameters
==========

TBD

Subscriptions
=============

 * /tts [std_msgs/String]

 * /engine [std_msgs/String]
 Among values "at", "espeak", "festival", "gnustep", "google", "ivona",
              "microsoft", "pico2wave", "speech_dispatcher"

Publications
============

TBD
