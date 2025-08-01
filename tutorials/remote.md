# How to Remote into the jetson

Before you can remote into the Jetson, you will need to do a little setup.
Along with that, you will need to make a hotspot. This is due to the fact that
on eduroam, SSH and other remote protocols are tough to use for self-hosted devices.
This may be fixed in the future, but I can't make any guarantees.

## Things you will need for the inital remote setp

1. a host device from which to remote into the jetson
2. the ability to make a hot spot on your phone and sufficet data
3. the jetson itself
4. monitor, keyboard, mouse and dp cable

## Host setup

1. go to the nomachine website and download and install the correct version for you os. [link to website](https://download.nomachine.com/everybody/)

## Jetson setup

1. plug the keyboard, mouse and dp into the jetson
2. plug in the battary into the car or use the jetson wallplug to power it on
3. after boot, turn on your hotspot and connect the jetson and your laptop to it

## Final steps

1. go over to your other device and open the nomachine cliet
2. you should now see the jetson pop up on the nomachine cliet, connect to it and use the password from the discord
3. now you can disconecct the monitor, keyboard and mouse. (make sure that you plug in the dummy dp into the jetson, otherwise it will not work)
4. now can you remote into the jetson, as long as you are on the same network as it.
