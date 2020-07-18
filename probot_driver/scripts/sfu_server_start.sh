#!/bin/bash

# $ ./sfu_server_start.sh [sfu_client_ip] [flag_default_user_config_path]
# example. $ ./sfu_server_start.sh 127.0.0.1 0

DEFAULT_SFU_CLIENT_IP=192.168.2.123
DEFAULT_SFU_CLIENT_PORT=6666
USER_NAME=`whoami`
DEFAULT_USER_CONFIG_FILE_PATH="/home/${USER_NAME}/.ros"

client_ip=$DEFAULT_SFU_CLIENT_IP
user_config_path=$DEFAULT_USER_CONFIG_FILE_PATH

argc=$#
if [[ $argc -ge 1 ]]; then
	client_ip=$1
fi
if [[ $argc -ge 2 ]]; then
	flag_default_user_config_path=$2
	if [[ $flag_default_user_config_path == "0" ]]; then
		user_config_path=""
	fi
fi

# pwd # ~/.ros
sleep 5
cd $(rospack find probot_driver)/bin
./SFUServer ${client_ip} ${DEFAULT_SFU_CLIENT_PORT} ${user_config_path}