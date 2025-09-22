#! /bin/sh
sudo chown -R $(id -u):$(id -g) /MapCleaner
. /MapCleaner/install/setup.sh
ros2 launch map_cleaner run.launch config:=/data/config/config.yaml
