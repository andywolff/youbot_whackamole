#!/bin/bash
setmole()
{
  echo setting mole to pos $1
  rostopic pub /whackamole/molepos std_msgs/Int16 $1 -1 &
}


p=0
pos=1 #0=yellow 1=neither 2=brown mole up
t=0 #time since last 
pic=1 #whether i should take a picture this minute


#infinitely loop, cycling mole positions. 
#once every minute, take a picture of the yellow mole
while :
do
  setmole $pos

  echo time = $t
  tt=$(($t % 60))
  if [ $tt -lt 30 -a $pic -eq 1 -a $pos -eq 0 ]
  then
    echo a minute has passed, stabilizing to save image
    sleep 10
    t=$(($t+10))
    rosrun youbot_whackamole rclick_on_image.sh &>/dev/null
    echo image saved
    pic=0
  fi
  if [ $tt -gt 30 ]; then pic=1; fi

  sleep 4
  t=$(echo "$t+4" | bc)
  p=$(($p+1))
  p=$(($p % 4))
  if [ $p -eq 0 -o $p -eq 2 ]; then pos=1; fi
  if [ $p -eq 1 ]; then pos=2; fi;
  if [ $p -eq 3 ]; then pos=0; fi;

done
