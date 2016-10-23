picotts
=======

[![Build Status](https://travis-ci.org/arnaud-ramey/picotts.svg)](https://travis-ci.org/arnaud-ramey/picotts)

A lightweight ROS package for Text-to-Speech (TTS) generation.
It is actually a wrapper around popular TTS engines,
it supports a bunch of both online and offline third parties.

* Open-source third parties:

  - **espeak**
    based on the espeak utility, a multi-lingual software speech synthesizer
    [link](http://espeak.sourceforge.net/)
  - **Festival**
    based on the Festival project, by the University of Edinburgh
    [link](http://www.cstr.ed.ac.uk/projects/festival/)
  - **GNUstep**
    the TTS engine of the GNUstep project.
    In Lubuntu 16.04, seems to be a wrapper around festival.
    [link](http://savannah.gnu.org/projects/gnustep/)
  - **MaryTTS**
    An open-source, multilingual Text-to-Speech Synthesis platform written in Java.
    This package makes use of the web service that is shipped with MaryTTS 5.2.
    [download page](http://mary.dfki.de/download/)
    [direct v5.2 download link](https://github.com/marytts/marytts/releases/download/v5.2/marytts-5.2.zip)
  - **pico2wave**
    based on pico2wave, a small footprint TTS
    [link](https://en.wikipedia.org/wiki/SVOX)
  - **speech_dispatcher**
    Uses the ```spd-say``` tool of the Speech Dispatcher project,
    developed by Freebsoft.
    [link](https://devel.freebsoft.org/speechd-el)

* Proprietary engines:

  - **AT&T** (WEBSERVICE)
    AT&T TTS webservice
    [link](http://www2.research.att.com/~ttsweb/tts/)
  - **Google TTS** (WEBSERVICE)
    most notably used in Google Translate
    [link](https://en.wikipedia.org/wiki/Google_Text-to-Speech)
  - **Ivona** (WEBSERVICE)
    uses the IVONA TTS webservice for developpers
    [link](http://developer.ivona.com/en/speechcloud/index.html)
  - **Microsoft** (WEBSERVICE)
    the speech synthesizer developed by Microsoft for its OS
    [link](https://en.wikipedia.org/wiki/Microsoft_text-to-speech_voices)

Supported languages
===================

The two-letter codes correspond to the ISO 639-1 codes,
and must be passed as such as language parameters to ```picotts```.
Their meaning is [available on Wikipedia](https://en.wikipedia.org/wiki/List_of_ISO_639-1_codes).

  - AT&T (WEBSERVICE):
      ```en es```
  - espeak:
      ```af an bg bs ca cs cy da de el en en-gb en-sc en-uk-north en-uk-rp en-uk-wmids en-us en-wi eo es es-la et fa fa-pin fi fr-be fr-fr=fr ga grc hi hr hu hy hy-west id is it jbo ka kn ku la  lfn lt lv mk ml ms ne nl no pa pl pt-br pt-pt ro ru sk sq sr sv sw ta tr vi vi-hue vi-sgn zh zh-yue```
  - Festival:
      ```scots_gaelic castillian_spanish english italian finnish american_english russian british_english welsh czech```
  - GNUstep:
      ```en```
  - Google TTS (WEBSERVICE):
      ```af ar cs cy da en eo es fr hi hr hu hy ko la lv nl no pl sk sq vi```
  - Ivona (WEBSERVICE):
      ```cy-GB da-DK de-DE en-AU en-GB en-GB-WLS en-IN en-US es-ES es-US fr-CA fr-FR is-IS it-IT nb-NO nl-NL pl-PL pt-BR pt-PT ro-RO ru-RU sv-SE tr-TR```
  - MaryTTS:
      ```de en-GB en-US fr it lb ru sv te tr```.
      You must download the desired languages using ```marytts-component-installer```,
      shipped in the ```bin``` folder of MaryTTS.
  - Microsoft (WEBSERVICE):
    ```ar ar-eg ca ca-es da da-dk de de-de en en-au en-ca en-gb en-in en-us es es-es es-mx fi fi-fi fr fr-ca fr-fr hi hi-in it it-it ja ja-jp ko ko-kr nb-no nl nl-nl no pl pl-pl pt pt-br pt-pt ru ru-ru sv sv-se yue zh-chs zh-cht zh-cn zh-hk zh-tw```
  - pico2wave:
    ```en-US en-GB de-DE es-ES fr-FR it-IT```
  - speech_dispatcher:
    ```af an bg bs ca cs cy da de el en eo es et fa fi fr ga grc hi hr hu hy id is it jbo ka kn ku la lfn lt lv mk ml ms ne nl no pa pl pt ro ru sk sq sr sv sw ta tr vi zh```


Licence
=======

BSD


Authors
=======

  - Package maintainer: Arnaud Ramey (arnaud.a.ramey@gmail.com)

Compile and install
===================

Dependencies
------------

  - AT&T (WEBSERVICE):
```bash
$ rosdep install picotts --ignore-src
```
  - espeak:
```bash
$ rosdep install picotts --ignore-src
```
  - Festival:
```bash
$ sudo apt install mbrola mbrola-fr*
$ rosdep install picotts --ignore-src
```
  - GNUstep:
```bash
$ sudo apt install gnustep-gui-runtime
```
  - Google TTS (WEBSERVICE):
```bash
$ rosdep install picotts --ignore-src
```
  - Ivona (WEBSERVICE):
```bash
$ rosdep install picotts --ignore-src
```
  - MaryTTS:
```bash
$ sudo apt install default-jre
```
  - Microsoft (WEBSERVICE):
```bash
$ rosdep install picotts --ignore-src
```
  - pico2wave:
```bash
$ rosdep install picotts --ignore-src
```
  - speech_dispatcher:
```bash
$ sudo apt install speech-dispatcher
```


ROS Kinetic + catkin
-------------------

Compile with [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make):

```bash
$ rosdep install picotts --ignore-src
$ catkin_make --only-pkg-with-deps picotts
```

Run
===

```bash
$ rosrun picotts picotts.exe
```

Change engine:

```bash
$ rostopic pub /picotts/engine std_msgs/String "microsoft"
```

Samples:

```bash
$ rostopic pub /tts std_msgs/String "OK"
$ rostopic pub /tts std_msgs/String "en:Hello|en:Hi"
```

Sample: read time:

```bash
$ echo -e "data: \"`date +%T`\"\n---" | rostopic pub /tts std_msgs/String -l
```

Parameters
==========

 * ```~ivona_credentials``` [std_msgs/String]
  a text file containing two lines,
  the first being the access key, the second the secret key.

 * ```~engine``` [std_msgs/String]
  Among values  "at&t", "espeak", "festival", "gnustep", "google", "ivona", "marytts"
                "microsoft", "pico2wave", "speech_dispatcher".
  Ensure to choose a webservice engine only if connected to the Internet.

 * ```/language``` [std_msgs/String]
 Cf § "Supported languages".

Subscriptions
=============

 * ```/tts``` [std_msgs/String]
 Sentences to be said.

 * ```/engine``` [std_msgs/String]
 Allow changing the engine while running, without restarting ```picotts```.
 Cf. description of the ```engine``` parameter.

Publications
============

None.
