#!/usr/bin/env bash

#=================================================
#	System Required: Ubuntu 16.04
#	Description: Install ROS And PROBOT
#	Version: 1.0.0
#	Author: ps-micro
#	Site: http://www.ps-micro.com/
#=================================================

default_version="v1.0.0"

Green_font_prefix="\033[32m"
Red_font_prefix="\033[31m"
Green_background_prefix="\033[42;37m"
Red_background_prefix="\033[41;37m"
Font_color_suffix="\033[0m"

Info="${Green_font_prefix}[Info]${Font_color_suffix}"
Error="${Red_font_prefix}[Error]${Font_color_suffix}"
Tip="${Green_font_prefix}[Warn]${Font_color_suffix}"

OSVersion=$(lsb_release -r --short)
OSDescription=$(lsb_release -d --short)

check_system_version()
{
    if [[ "${OSVersion}"   == "14.04" ]]; then
        ROS_Ver="indigo"
    elif [[ "${OSVersion}" == "16.04" ]]; then
        ROS_Ver="kinetic"
    elif [[ "${OSVersion}" == "18.04" ]]; then
        ROS_Ver="melodic"
    else
       echo -e "${Error} PROBOT don't support ${OSDescription} !" && exit 1
    fi
}


check_ros_version()
{
	echo -e "${Tip} The system version: ${OSDescription}" 
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} The ROS version: ${ROSVER}"
			return
		fi 
	fi
	echo -e "${Error} ROS has not installed in this computer, please install ROS with 1." 
}

install_ros()
{
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} The ROS ${ROSVER} has been installed!" 
			echo && stty erase ^? && read -p "Do you want to continue？ y/n：" choose
			if [[ "${choose}" == "y" ]]; then
				echo -e "${Info}Start to install ROS！" 
			else
				exit
			fi
		fi
	fi

    echo -e "${Green_font_prefix}Install ROS ...${Font_color_suffix}"

	sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116		
	sudo apt-get -y update
	sudo apt-get -y install ros-${ROS_Ver}-desktop-full
	sudo rosdep init
	rosdep update
	echo "source /opt/ros/${ROS_Ver}/setup.bash" >> ~/.bashrc
	source /opt/ros/${ROS_Ver}/setup.bash
	sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

    echo -e "${Green_font_prefix}ROS has installed finished.${Font_color_suffix}"
}


install_probot_dependents()
{
    echo -e "${Green_font_prefix}Install probot dependent packages ...${Font_color_suffix}"

    sudo apt-get -y install ros-${ROS_Ver}-moveit-*
    sudo apt-get -y install ros-${ROS_Ver}-industrial-*
    sudo apt-get -y install ros-${ROS_Ver}-gazebo-ros-control
    sudo apt-get -y install ros-${ROS_Ver}-ros-control ros-${ROS_Ver}-ros-controllers
    sudo apt-get -y install ros-${ROS_Ver}-trac-ik-kinematics-plugin
    sudo apt-get -y install ros-${ROS_Ver}-usb-cam

    echo -e "${Green_font_prefix}Dependent packages have installed finished.${Font_color_suffix}"
}


install_probot()
{
    echo -e "${Green_font_prefix}Install probot packages ...${Font_color_suffix}"

    if [ ! -d ~/probot_anno_ws ]; then
        mkdir ~/probot_anno_ws
        mkdir ~/probot_anno_ws/src
        mkdir ~/probot_anno_ws/src/probot_anno
        cp -R * ~/probot_anno_ws/src/probot_anno
        
        chmod +x ~/probot_anno_ws/src/probot_anno/probot_demo/scripts/*
        chmod +x ~/probot_anno_ws/src/probot_anno/probot_driver/bin/*
        chmod +x ~/probot_anno_ws/src/probot_anno/probot_driver/scripts/*

        cd ~/probot_anno_ws
        catkin_make

	    echo "source ~/probot_anno_ws/devel/setup.bash --extend" >> ~/.bashrc

	    source ~/.bashrc

        echo -e "${Green_font_prefix}PROBOT has installed to $(pwd) 
    Please have a happy journey!${Font_color_suffix}"
    else
        echo -e "${Red_font_prefix}The probot_anno_ws folder has existed, please delete and reinstall!${Font_color_suffix}"
    fi
}

install_all_environment()
{
    install_ros
    install_probot_dependents
    install_probot
}

main()
{
    check_system_version

    echo -e "PROBOT Setup Assistant${Red_font_prefix}[${default_version}]${Font_color_suffix}
---- www.ps-micro.com ----

${Green_font_prefix}  0.${Font_color_suffix} Install ROS
${Green_font_prefix}  1.${Font_color_suffix} Install PROBOT dependent packages
${Green_font_prefix}  2.${Font_color_suffix} Install PROBOT packages
-------------------
${Green_font_prefix}  3.${Font_color_suffix} Install all environment
    "
    check_ros_version

    echo && stty erase ^? && read -p "please choose [0~3]：" num
    case "$num" in
	    0)
	    install_ros
	    ;;
	    1)
	    install_probot_dependents
	    ;;
	    2)
	    install_probot
	    ;;
	    3)
	    install_all_environment
	    ;;
	    *)
	    echo -e "${Error} please input right selection. "
	    ;;
    esac   
}

main $*
