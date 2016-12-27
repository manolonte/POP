#!/bin/sh

n=`ls -x *.mp3 | wc -l`

file=`ls -x  *.mp3 | awk -v min=1 -v max=$n 'BEGIN{srand(); nrand=int(min+rand()*(max-min+1))} {nrand--; if (nrand == 0) print $0 } '`
echo $file
export AUDIODEV=pcm.jack
sox "$file" -t alsa fade h 1 6 2 gain -20
killall silentjack
