#!/bin/sh

silence_threshold=$1

./silentjack -c system:capture_1 -l -${silence_threshold} -g 1 -p 0.5 ./ovation.sh
