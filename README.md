
# DEBI Robotics Challenge

    

## Installation of Important Tools

Tools used :
```
    Ubuntu 20.04, 
    VMware player 17,
    Gazebo Classic,
    ROS 1 Noetic,
    Turtlebot3 Toolbox
```

1- Download VMware Workstation17 
```
https://customerconnect.vmware.com/en/downloads/details?downloadGroup=WKST-PLAYER-1701&productId=1377&rPId=100675
```
2- Download Ubuntu 20.04 LTS
```
https://releases.ubuntu.com/focal/ubuntu-20.04.6-desktop-amd64.iso
```
3- Setting up vmware
```
After installing vmware and downloading ubuntu iso file do the following:
Open Vmware -> Create new virtual machine -> click browse and choose the iso file you've downloaded -> continue with the rest of the step normally without changing any defaults.

Run ubuntu x64 and continue with the rest of the setup with default settings.
```
4- Clone repository
```
After installing Ubuntu, open the terminal by pressing (ctrl+shift+t) and paste the following:
git clone https://github.com/mohamedsafy/robotics_challenge
and hit enter
```
5- Run install-script.sh
```
cd robotics_challenge
./install-script.sh
```
wait for all the dependencies to be installed.

6- Test
```
Paste this to your terminal
export TURTLEBOT3_MODEL=waffle_pi

roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
If you don't see gazebo openning, please contact me.

##Start writing code!
Copy robotics_challenge sub folder to /catkin_ws/src/
Changing the code inside .py files doesn't require compile but if you add new files you have to run
```
cd ~/catkin_ws/ && catkin_make
```
```
Launch our simulation by running:
roslaunch robotics_challenge start.launch
```

## Files Structure
```
We have three main nodes connected with eachother using combination of topics and actions.
The code template is written in a way that allows each group to work and test their code individually from other groups. 
Many of the functions is made as a mockup, code should be replaced with every group implemention.
```
