#!/bin/bash

# Controller's IP
DEFAULT_SFU_CLIENT_IP=192.168.2.123
DEFAULT_SFU_CLIENT_PORT=6666

# pwd # ~/.ros
sleep 5
cd $(rospack find probot_driver)/bin
./SFUServer ${DEFAULT_SFU_CLIENT_IP} ${DEFAULT_SFU_CLIENT_PORT}
