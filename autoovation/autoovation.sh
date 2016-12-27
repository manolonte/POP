#!/bin/sh

/usr/local/bin/led_off.sh
sleep 1
/usr/local/bin/led_on.sh
noise_threshold=$1
silence_threshold=$2
cd /home/chip/autoovation
while [ 1 ]
do
  ./silentjack -n noisyjack -r -c system:capture_1 -l -${noise_threshold} -g 2 -p 2 ./prepare_for_ovation.sh ${silence_threshold}
done
