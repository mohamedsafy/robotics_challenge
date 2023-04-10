# robotics_challenge

This repo will be public untill all participance make a github account, then they will be added as a contributors to the repo.

Setting up the Working space
Ubuntu 20.04 + ROS Noetic + gazebo will be used and you will be guided on how to setup all that.

Part 1 : Installing Ubuntu

  Step 1: Install VMware
  
    1- Head to VMware Workstation player [Download Page](https://customerconnect.vmware.com/en/downloads/details?downloadGroup=WKST-PLAYER-1701&productId=1377&rPId=100675)
    
    2- Continue installing VMware normally
    
  Step 2: Install Ubuntu
  
    1- Download this exact version of Ubuntu [here](https://releases.ubuntu.com/focal/ubuntu-20.04.6-desktop-amd64.iso)
    
    2-Open VMware -> Create new Virtual Machine 
    
    3- Choose the iso file you've downloaded in point 1, by pressing browse under Installer disk image file(iso)
    
    4- Choose username and password 
    
    5- continue with installation normally
    
    6- double click ubuntu 64-bit or click the green triangle to start the Virtual Machine
    
    7- continue with the installation normally with the default configurations.
    
    **PLEASE DO NOT CARRY ON WITH THE REST OF THE INSTRUCTIONS. UPDATES WILL BE MADE SOON**
Part 2: Install ROS + gazebo
  
  I've compiled all the nessasary steps into one simple script, all you have to do is copy it to your virtual machine and run it as such:
  
  1-Open Terminal (Ctrl+alt+T).
  
  2-Paste the following by pressing (Ctrl+Shift+V)
  
   git clone https://github.com/mohamedsafy/robotics_challenge.git
   
  and hit Enter.
  
  Then move the script.sh to your home directory.
  
  run the script.

Part 3: Test
  
  Same as the previous step but paste this :
  
  export TURTLEBOT3_MODEL=waffle_pi 
  
  roslaunch turtlebot3_gazebo turtlebot3_house.launch
  
Your workspace is now ready for simulation.

To be continued....
  
