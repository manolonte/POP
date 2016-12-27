#!/bin/sh

# Kernel 4.3
# 408 - LED
# 409 - S1
# 410 - S2
# 411 - B1 ROJO -> RESET/SHUTDOWN
# 412 - B2

mode=$1
mode_orig=$mode

while [ 1 ]
do
   sleep 1
   v411=`cat /sys/class/gpio/gpio411/value`
   v409=`cat /sys/class/gpio/gpio409/value`
   v410=`cat /sys/class/gpio/gpio410/value`
   v412=`cat /sys/class/gpio/gpio412/value`

   # echo $v411
   if [ $v411 -eq 1 ]
   then
      amixer -c 0 cset numid=7 off
      sleep 5
      v411=`cat /sys/class/gpio/gpio411/value`
      if [ $v411 -eq 1 ]
      then
         shutdown -h 0
      else
         shutdown -r 0
      fi
      while [ 1 ]
      do
	sleep 1
        echo 1 > /sys/class/gpio/gpio408/value
        sleep 1
	echo 0 > /sys/class/gpio/gpio408/value
      done
   fi
   if [ $v412 -eq 1 ]
   then
      # Black button pressed
      echo "Black button " >> /var/log/pop.log
      runuser -l chip /home/chip/autoovation/autoovation.sh 20 20 &
   fi
   if [ "$v409" -eq 1 ] && [ "$v410" -eq 0 ]
   then
      mode=mnt
   elif [ "$v409" -eq 1 ] && [ "$v410" -eq 1 ]
   then
      mode=p2
   elif [ "$v409" -eq 0 ] && [ "$v410" -eq 1 ]
   then
      mode=p1
   fi
   if [ "$mode" = "p1" ]
   then
      pop_script="pop_p1.sh"
   fi
   if [ "$mode" = "p2" ]
   then
      pop_script="pop_p2.sh"
   fi
   if [ "$mode" = "mnt" ]
   then
      pop_script="pop_mnt.sh"
   fi

   if [ "$mode" != "$mode_orig" ]
   then
      mode_orig=$mode
      echo 1 > /sys/class/gpio/gpio408/value
      killall jackd
      killall mod-host
      killall a2jmidi_bridge
      pkill -f ".*autoovation.*" 
      /sbin/runuser -l chip /home/chip/${pop_script} >> /var/log/pop.log 2>&1 &
   fi
done
