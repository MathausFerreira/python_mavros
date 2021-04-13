#!/bin/sh
ls

# Salva Variável para Github
echo "What is your GitHub Name"
read GitName
echo "What is your GitHub E-mail"
read GitMail

# Install GitHub

sudo apt-get update -y
sudo apt-get install git -y
sudo apt-get install gitk git-gui -y

git config --global user.name "$GitName"
git config --global user.email "$GitMail"

cd Documents
mkdir Github
cd Github

#google chrome
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo apt install ./google-chrome-stable_current_amd64.deb -y

#Cloning PX4-Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
#setting up Px4-Autopilot
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
# Setting up ROS melodic
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
bash ubuntu_sim_ros_melodic.sh
bash ubuntu_sim_ros_melodic.sh

#Clonning Ardupilot_2020 (grin)
git clone https://github.com/MathausFerreira/Ardupilot_2020.git
bash Tools/environment_install/install-prereqs-ubuntu.sh -y
#. ~/.profile
