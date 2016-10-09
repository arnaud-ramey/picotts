#!/bin/bash
##########################################################################
# MARY TTS server
##########################################################################

if [ $# -lt 1 ]; then
  echo "Launch Mary TTS as server."
  echo "Synopsis: ivona.bash MARY_BASE"
  echo " where MARY_BASE is the folder containing bin, lib, etc."
  exit -1
fi

# Set the Mary base installation directory in an environment variable:
export MARY_BASE=$1
java -showversion -Xms40m -Xmx1g -cp "$MARY_BASE/lib/*" -Dmary.base="$MARY_BASE"  marytts.server.Mary
