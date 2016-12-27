#!/bin/sh
set -x
# Kernel 4.3
# 408 - LED
# 409 - S1 - posición abajo
# 410 - S2 - posición arriba
# 411 - B1 ROJO -> RESET/SHUTDOWN
# 412 - B2 NEGRO

echo 408 > /sys/class/gpio/export 
echo out > /sys/class/gpio/gpio408/direction
echo 1 > /sys/class/gpio/gpio408/value
echo 409 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio409/direction
echo 410 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio409/direction
echo 411 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio409/direction
echo 412 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio409/direction

v409=`cat /sys/class/gpio/gpio409/value`
v410=`cat /sys/class/gpio/gpio410/value`
echo $v409
echo $v410

# 408=1 409=0 -> Mantenimiento, switch abajo
# 408=1 409=1 -> Switch enmedio
# 408=0 409=1 -> Switch arriba

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
echo $mode

if [ "$mode" != "mnt" ]
then
   echo "iface wlan0 inet manual" > /etc/network/interfaces
else
   :> /etc/network/interfaces   
fi

/usr/local/bin/pop_loop.sh $mode &

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
date >> /var/log/pop.log
echo 1 > /sys/class/gpio/gpio408/value
chmod 777 /sys/class/gpio/gpio408/value
/sbin/runuser -l chip /home/chip/${pop_script} >> /var/log/pop.log 2>&1 &

