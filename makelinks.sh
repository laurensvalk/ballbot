#!/usr/bin/env bash
rm -rf ev3devices
mkdir ev3devices
ls -d /sys/class/*/{sensor,motor}*/ | while read devdir
   do cat $devdir"address" | while read port
      do  rm -f $port
      ln -s /$devdir ev3devices/$port
   done
done
