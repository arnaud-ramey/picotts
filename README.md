picotts
=======

A lightweight ROS package for Text-to-Speech generation.

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
